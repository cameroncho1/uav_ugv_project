# Per-channel specialist models for the multivariate UGV layout.
#
# Each of the G faultable channels gets its own small network that sees only that
# channel's feature column and outputs only that channel's decision. The G networks
# are packed into one wide layer whose weights are masked to a block-diagonal
# pattern: weights between different channels start at zero and receive zero
# gradient, so they stay zero and the blocks never exchange information. The result
# equals G separately built small networks (test_training.py checks this), but runs
# as one fused recurrent kernel, which is 3-4x faster on the Apple GPU.
#
# Trained together, the blocks share batches, the stopping epoch, and gradient-norm
# clipping; their weights do not interact.

import math

import torch
import torch.nn as nn
import torch.nn.functional as F


def _block_mask(rows, cols, row_group, col_group):
    # 1 where the row unit and the column unit belong to the same channel.
    r = torch.tensor([row_group(i) for i in range(rows)])
    c = torch.tensor([col_group(j) for j in range(cols)])
    return (r[:, None] == c[None, :]).float()


def _mask_parameter(param, mask, bound):
    # Re-initialize in-block weights as a small network of this width would be, zero
    # the rest, and zero their gradients so the optimizer never moves them. Returns
    # the number of weights held at zero.
    with torch.no_grad():
        param.uniform_(-bound, bound).mul_(mask)
    param.register_hook(lambda grad, m=mask: grad * m.to(grad.device))
    return int(mask.numel() - mask.sum())


class _BlockRecurrent(nn.Module):
    # G small recurrent networks (2 layers, hidden h each) as one masked nn.LSTM/nn.GRU.
    def __init__(self, cell, columns, hidden, num_layers, bidirectional):
        super().__init__()
        self.columns = list(columns)
        self.groups, self.hidden = len(self.columns), hidden
        width = self.groups * hidden
        self.width = width
        self.rnn = cell(self.groups, width, num_layers=num_layers, batch_first=True,
                        bidirectional=bidirectional)
        unit = lambda i: (i % width) // hidden            # channel of a hidden unit
        bound = 1.0 / math.sqrt(hidden)
        self.fixed_zeros = 0
        for name, param in self.rnn.named_parameters():
            if name.startswith("bias"):
                with torch.no_grad():
                    param.uniform_(-bound, bound)
                continue
            rows, cols = param.shape
            first_input = name.startswith("weight_ih_l0")
            col_group = (lambda j: j) if first_input else unit
            self.fixed_zeros += _mask_parameter(param, _block_mask(rows, cols, unit, col_group),
                                                bound)

    def channels(self, x):
        return x[:, :, self.columns]                      # (batch, L, G)


class BlockBiLSTM(_BlockRecurrent):
    # Per-channel BiLSTMs with the 'full' readout of models/BiLstm.py.
    def __init__(self, columns, hidden=15, num_layers=2):
        super().__init__(nn.LSTM, columns, hidden, num_layers, bidirectional=True)
        self.fc = nn.Linear(2 * self.width, self.groups)
        unit = lambda i: (i % self.width) // hidden
        self.fixed_zeros += _mask_parameter(
            self.fc.weight, _block_mask(self.groups, 2 * self.width, lambda k: k, unit),
            1.0 / math.sqrt(2 * hidden))
        with torch.no_grad():
            self.fc.bias.uniform_(-1.0 / math.sqrt(2 * hidden), 1.0 / math.sqrt(2 * hidden))

    def forward(self, x):
        out, _ = self.rnn(self.channels(x))
        features = torch.cat((out[:, -1, :self.width], out[:, 0, self.width:]), dim=1)
        return self.fc(features)

    modelName = "BiLSTM specialists"


class BlockGRU(_BlockRecurrent):
    # Per-channel GRUs reading out the last step, as models/Gru.py.
    def __init__(self, columns, hidden=15, num_layers=2):
        super().__init__(nn.GRU, columns, hidden, num_layers, bidirectional=False)
        self.fc = nn.Linear(self.width, self.groups)
        self.fixed_zeros += _mask_parameter(
            self.fc.weight, _block_mask(self.groups, self.width, lambda k: k, lambda i: i // hidden),
            1.0 / math.sqrt(hidden))
        with torch.no_grad():
            self.fc.bias.uniform_(-1.0 / math.sqrt(hidden), 1.0 / math.sqrt(hidden))

    def forward(self, x):
        out, _ = self.rnn(self.channels(x))
        return self.fc(out[:, -1, :])

    modelName = "GRU specialists"


class GroupedAutoencoder(nn.Module):
    # G small autoencoders, one per channel: its L-sample column -> hidden ->
    # bottleneck -> L samples, LeakyReLU after the hidden layer and the bottleneck,
    # linear output. Input and output are flattened (L * F) windows as for
    # models/Autoencoder.py; columns that are not fault targets are passed through,
    # so their reconstruction error is zero.
    def __init__(self, columns, window_len, num_features, hidden=128, bottleneck=4):
        super().__init__()
        self.columns = list(columns)
        self.window_len, self.num_features = window_len, num_features
        groups = len(self.columns)
        sizes = [window_len, hidden, bottleneck, window_len]
        self.weights = nn.ParameterList()
        self.biases = nn.ParameterList()
        for size_in, size_out in zip(sizes[:-1], sizes[1:]):
            bound = 1.0 / math.sqrt(size_in)              # nn.Linear's default range
            self.weights.append(nn.Parameter(torch.empty(groups, size_in, size_out).uniform_(-bound, bound)))
            self.biases.append(nn.Parameter(torch.empty(groups, size_out).uniform_(-bound, bound)))

    def forward(self, x):
        window = x.reshape(x.shape[0], self.window_len, self.num_features)
        h = window[:, :, self.columns].transpose(1, 2)    # (batch, G, L)
        last = len(self.weights) - 1
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            h = torch.einsum("bgi,gio->bgo", h, w) + b
            if i < last:
                h = F.leaky_relu(h)
        columns = list(window.unbind(dim=2))
        for k, column in enumerate(self.columns):
            columns[column] = h[:, k, :]
        return torch.stack(columns, dim=2).flatten(start_dim=1)

    modelName = "Autoencoder specialists"


def effective_parameters(model):
    # Trainable weights, excluding the masked weights held at zero.
    return sum(p.numel() for p in model.parameters()) - getattr(model, "fixed_zeros", 0)
