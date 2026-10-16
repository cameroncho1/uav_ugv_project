# Autoencoder model for fault detection in UAV and UGV systems
import os

import json

import torch
import torch.nn as nn


# Load default model configuration from JSON file, relative to this module so it
# can be imported from any working directory.
with open(os.path.join(os.path.dirname(__file__), 'utils', 'ModelParameters.json'), 'r', encoding='utf-8') as f:
    config = json.load(f)
f.close()

class Autoencoder(nn.Module):
    def __init__(self, input_dim = config['shared']['input_dim'], hidden_dim = config['autoencoder']['hidden_dim']):
        super(Autoencoder, self).__init__()
        self.encoder = nn.Linear(input_dim, hidden_dim)
        self.decoder = nn.Linear(hidden_dim, input_dim)

    def forward(self, x):
        encoded = torch.relu(self.encoder(x))
        decoded = torch.sigmoid(self.decoder(encoded))
        return decoded

    def encode(self, x):
        return torch.relu(self.encoder(x))

    def decode(self, encoded):
        return torch.sigmoid(self.decoder(encoded))

    def reconstruct(self, x):
        encoded = self.encode(x)
        decoded = self.decode(encoded)
        return decoded

    modelName = "Autoencoder"

