# Checks for the Trainer options and the UGV split rules.
# Run with ``./bin/python test_training.py`` (also pytest-compatible).

import json
import os
import tempfile
from pathlib import Path

import torch
from torch import nn

from models.Autoencoder import Autoencoder
from models.BiLstm import BiLSTM
from models.Gru import GRU
from models.Specialists import BlockBiLSTM, BlockGRU, GroupedAutoencoder
from models.Trainer import Trainer
from ugv_dataset import variant_splits


class Recording(torch.optim.SGD):
    # SGD that records the gradient norm it is asked to apply.
    norms = []

    def step(self, closure=None):
        grads = [p.grad.flatten() for g in self.param_groups for p in g["params"]
                 if p.grad is not None]
        Recording.norms.append(torch.cat(grads).norm().item())
        return super().step(closure)


def tiny_data(n=64):
    x = torch.randn(n, 4)
    return torch.utils.data.TensorDataset(x, 100 * x.sum(dim=1, keepdim=True))


def test_patience_stops_and_saves_final_model():
    with tempfile.TemporaryDirectory() as directory:
        model = nn.Linear(4, 1)
        # lr=0 keeps the validation loss constant, so epoch 1 stays the best.
        trainer = Trainer(model, tiny_data(), tiny_data(), nn.MSELoss(),
                          torch.optim.SGD(model.parameters(), lr=0.0), batch_size=16,
                          num_epochs=50, save_dir=directory, save_every=1000, patience=3)
        trainer.train()
        assert len(trainer.history["train"]) == 4 and trainer.best_epoch == 1
        assert os.path.exists(os.path.join(directory, "final_model.pt"))


def test_grad_clip_bounds_each_step():
    model = nn.Linear(4, 1)
    Recording.norms = []
    Trainer(model, tiny_data(), None, nn.MSELoss(), Recording(model.parameters(), lr=1e-6),
            batch_size=16, num_epochs=1, grad_clip=0.5).train()
    assert Recording.norms and max(Recording.norms) <= 0.5 + 1e-5


def test_gru_outputs_one_logit_per_channel():
    assert GRU(input_dim=15, hidden_dim=8, output_dim=11, num_layers=2)(
        torch.zeros(3, 100, 15)).shape == (3, 11)


def test_single_layer_autoencoder_keeps_checkpoint_keys():
    legacy = Autoencoder(input_dim=1500, hidden_dim=50, encoder_dims=[], decoder_dims=[],
                         activation="relu")
    assert set(legacy.state_dict()) == {"encoder.weight", "encoder.bias",
                                        "decoder.weight", "decoder.bias"}


def test_hidden_encoder_layer_and_leaky_relu():
    model = Autoencoder(input_dim=750, hidden_dim=50, encoder_dims=[60], decoder_dims=[],
                        activation="leaky_relu")
    shapes = [tuple(p.shape) for n, p in model.named_parameters() if n.endswith("weight")]
    assert shapes == [(60, 750), (50, 60), (750, 50)]
    x = torch.randn(4, 750, requires_grad=True)
    assert model(x).shape == (4, 750)
    # LeakyReLU passes gradient for negative inputs, so no unit can stop learning.
    z = torch.full((1, 3), -1.0, requires_grad=True)
    model.activation(z).sum().backward()
    assert (z.grad > 0).all()


def test_saved_autoencoder_layout_falls_back_to_legacy():
    import main
    with tempfile.TemporaryDirectory() as directory:
        assert main._saved_ae_layout(directory)["encoder_dims"] == []
        layout = {"hidden_dim": 50, "encoder_dims": [60], "decoder_dims": [],
                  "activation": "leaky_relu"}
        Path(directory, "model_config.json").write_text(json.dumps({"autoencoder": layout}))
        assert main._saved_ae_layout(directory) == layout


def test_autoencoder_layer_overrides():
    import types
    import main
    args = types.SimpleNamespace(ae_encoder_dims=[60], ae_decoder_dims=[60])
    layout = main._ae_layout(args)
    assert layout["encoder_dims"] == [60] and layout["decoder_dims"] == [60]
    assert main._ae_layout(types.SimpleNamespace(ae_encoder_dims=None, ae_decoder_dims=[]))[
        "decoder_dims"] == []


COLUMNS = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8, 10]   # main/ugv_multivariate TARGET_FEATURES


def block_rows(gates, width, hidden, k):
    # Rows of channel k's units in every gate chunk of a packed recurrent weight.
    return torch.cat([torch.arange(g * width + k * hidden, g * width + (k + 1) * hidden)
                      for g in range(gates)])


def unpack(block, small, gates, k, bidirectional):
    # Copy channel k's block of a packed model into a separately built small model.
    width, hidden = block.width, block.hidden
    rows = block_rows(gates, width, hidden, k)
    own = torch.arange(k * hidden, (k + 1) * hidden)
    pair = torch.cat([own, own + width]) if bidirectional else own
    state = {}
    for name, param in block.rnn.named_parameters():
        if name.startswith("weight_ih_l0"):
            value = param[rows][:, [k]]
        elif name.startswith("weight_ih"):
            value = param[rows][:, pair]
        elif name.startswith("weight_hh"):
            value = param[rows][:, own]
        else:
            value = param[rows]
        state[name] = value
    small_rnn = small.lstm if bidirectional else small.gru
    small_rnn.load_state_dict(state)
    small.fc.weight.data = block.fc.weight[[k]][:, pair].detach().clone()
    small.fc.bias.data = block.fc.bias[[k]].detach().clone()


def test_packed_specialists_equal_separate_small_networks():
    x = torch.randn(5, 40, 15)
    for block, make, gates, bi in (
            (BlockBiLSTM(COLUMNS, hidden=4), lambda: BiLSTM(input_dim=1, hidden_dim=4, output_dim=1,
                                                             num_layers=2), 4, True),
            (BlockGRU(COLUMNS, hidden=4), lambda: GRU(input_dim=1, hidden_dim=4, output_dim=1,
                                                       num_layers=2), 3, False)):
        packed = block(x)
        for k, column in enumerate(COLUMNS):
            small = make()
            unpack(block, small, gates, k, bi)
            assert torch.allclose(small(x[:, :, [column]])[:, 0], packed[:, k], atol=1e-5)


def test_masked_weights_stay_zero_after_training_steps():
    block = BlockBiLSTM(COLUMNS, hidden=4)
    zeros = {n: (p == 0).clone() for n, p in block.named_parameters() if p.dim() > 1}
    optimizer = torch.optim.Adam(block.parameters(), lr=0.01)
    for _ in range(3):
        optimizer.zero_grad()
        block(torch.randn(8, 30, 15)).pow(2).sum().backward()
        optimizer.step()
    for name, param in block.named_parameters():
        if name in zeros:
            assert (param[zeros[name]] == 0).all(), name


def test_grouped_autoencoder_keeps_channels_apart():
    model = GroupedAutoencoder(COLUMNS, window_len=20, num_features=15, hidden=8, bottleneck=2)
    x = torch.randn(3, 20, 15)
    base = model(x.flatten(1)).reshape(3, 20, 15)
    other = [c for c in range(15) if c not in COLUMNS]
    assert torch.equal(base[:, :, other], x[:, :, other])      # non-targets pass through
    bumped = x.clone()
    bumped[:, :, COLUMNS[4]] += 5.0
    changed = (model(bumped.flatten(1)).reshape(3, 20, 15) - base).abs().sum(dim=(0, 1)) > 0
    assert changed.tolist() == [c == COLUMNS[4] for c in range(15)]


def write_variants(folder, runs):
    for run in runs:
        variant = Path(folder) / f"run_{run}_noise_0"
        variant.mkdir(parents=True)
        (variant / "faults.json").write_text(json.dumps({"source_run": f"run_{run}"}))


def test_extra_validation_variants_must_come_from_validation_drives():
    with tempfile.TemporaryDirectory() as directory:
        base = Path(directory) / "base"
        write_variants(base, range(1000, 1006))
        splits = variant_splits(base, 2, 2, seed=0)
        val_runs = sorted(v.name.split("_")[1] for v in splits["val"])
        write_variants(Path(directory) / "good", val_runs)
        extended = variant_splits(base, 2, 2, seed=0, extra_val_dirs=[Path(directory) / "good"])
        assert len(extended["val"]) == 4 and extended["train"] == splits["train"]
        train_run = splits["train"][0].name.split("_")[1]
        write_variants(Path(directory) / "bad", [train_run])
        try:
            variant_splits(base, 2, 2, seed=0, extra_val_dirs=[Path(directory) / "bad"])
        except ValueError as error:
            assert "outside the validation split" in str(error)
        else:
            raise AssertionError("a training drive was accepted as validation")


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
