# RNN model for fault detection in UAV and UGV systems

import os

import json

import torch
import torch.nn as nn

# Load model configuration from JSON file.
# Resolve the path relative to this module so the model can be imported from any
# working directory (e.g. when main.py at the project root imports it).
with open(os.path.join(os.path.dirname(__file__), 'utils', 'ModelParameters.json'), 'r', encoding='utf-8') as f:
    config = json.load(f)
f.close()

class RNN(nn.Module):

    def __init__(self, input_dim = config['shared']['input_dim'], hidden_dim = config['rnn']['hidden_dim'], output_dim = config['shared']['output_dim'], num_layers = config['rnn']['num_layers']):
        super(RNN, self).__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.rnn = nn.RNN(input_dim, hidden_dim, num_layers=num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        # x shape: (batch, seq_len, input_dim)
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_dim)
        out, _ = self.rnn(x, h0)
        out = self.fc(out[:, -1, :])
        return out

    modelName = "RNN"

    