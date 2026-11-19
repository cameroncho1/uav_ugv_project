# GRU model for fault detection in UAV and UGV systems

import os

import json

import torch
import torch.nn as nn

# Load model configuration from JSON file, relative to this module.
with open(os.path.join(os.path.dirname(__file__), 'utils', 'ModelParameters.json'), 'r', encoding='utf-8') as f:
    config = json.load(f)


class GRU(nn.Module):
    # Unidirectional gated recurrent classifier. It replaces the plain RNN (models/Rnn.py),
    # whose tanh recurrence could not fit the 100-step multivariate windows; the
    # update and reset gates keep gradients usable across the window.

    def __init__(self, input_dim = config['shared']['input_dim'], hidden_dim = config['gru']['hidden_dim'], output_dim = config['shared']['output_dim'], num_layers = config['gru']['num_layers']):
        super(GRU, self).__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.gru = nn.GRU(input_dim, hidden_dim, num_layers=num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        # x shape: (batch, seq_len, input_dim); the last step's state has read the whole window.
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_dim, device=x.device)
        out, _ = self.gru(x, h0)
        return self.fc(out[:, -1, :])

    modelName = "GRU"
