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

ACTIVATIONS = {"relu": nn.ReLU, "leaky_relu": nn.LeakyReLU}
# Layout of checkpoints trained before the hidden encoder layer was added
# (folders whose model_config.json has no "autoencoder" entry).
LEGACY = {"encoder_dims": [], "decoder_dims": [], "activation": "relu"}


def _stack(sizes, activation):
    # Linear layers through ``sizes`` with the activation between them; a single
    # Linear stays a bare nn.Linear so its state_dict keys match older checkpoints.
    layers = []
    for i, (size_in, size_out) in enumerate(zip(sizes[:-1], sizes[1:])):
        if i:
            layers.append(activation())
        layers.append(nn.Linear(size_in, size_out))
    return layers[0] if len(layers) == 1 else nn.Sequential(*layers)


class Autoencoder(nn.Module):
    # Fully connected autoencoder over a flattened window:
    #   input -> encoder_dims... -> hidden_dim (bottleneck) -> decoder_dims... -> input
    # with ``activation`` after every hidden layer and the bottleneck; the output is
    # linear because standardized inputs can be negative. With no hidden layers and
    # 'relu' this is the original single-layer network.
    def __init__(self, input_dim = config['shared']['input_dim'], hidden_dim = config['autoencoder']['hidden_dim'],
                 encoder_dims = config['autoencoder'].get('encoder_dims', []),
                 decoder_dims = config['autoencoder'].get('decoder_dims', []),
                 activation = config['autoencoder'].get('activation', 'relu')):
        super(Autoencoder, self).__init__()
        if activation not in ACTIVATIONS:
            raise ValueError(f"activation must be one of {sorted(ACTIVATIONS)}, not {activation!r}")
        act = ACTIVATIONS[activation]
        self.encoder = _stack([input_dim, *encoder_dims, hidden_dim], act)
        self.activation = act()
        self.decoder = _stack([hidden_dim, *decoder_dims, input_dim], act)

    def forward(self, x):
        return self.decoder(self.encode(x))

    def encode(self, x):
        return self.activation(self.encoder(x))

    def decode(self, encoded):
        return self.decoder(encoded)

    def reconstruct(self, x):
        return self.decode(self.encode(x))

    modelName = "Autoencoder"

