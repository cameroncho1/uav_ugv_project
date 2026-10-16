# Bidirectional LSTM for fault detection

import os

import json

import torch
import torch.nn as nn

# Load model configuration from JSON file, relative to this module so it can be
# imported from any working directory.
with open(os.path.join(os.path.dirname(__file__), 'utils', 'ModelParameters.json'), 'r', encoding='utf-8') as f:
    config = json.load(f)
f.close()

class BiLSTM(nn.Module):
    def __init__(self, input_dim = config['shared']['input_dim'], hidden_dim = config['bilstm']['hidden_dim'], output_dim = config['shared']['output_dim'], num_layers = config['bilstm']['num_layers']):
        super(BiLSTM, self).__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers=num_layers, bidirectional=True, batch_first=True)
        self.fc = nn.Linear(hidden_dim * 2, output_dim)   # Multiply by 2 for bidirectional

    def forward(self, x):
         # x shape: (batch, seq_len, input_dim)
        h0 = torch.zeros(self.num_layers * 2, x.size(0), self.hidden_dim)   # Multiply by 2 for bidirectional
        c0 = torch.zeros(self.num_layers * 2, x.size(0), self.hidden_dim)   # Multiply by 2 for bidirectional
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

    modelName = "BiLSTM"
