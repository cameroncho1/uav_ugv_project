# Training script for fault detection models
# Can use this script to train various models such as RNN, LSTM, GRU, and Autoencoder for fault detection in UAV and UGV systems

import os
import json
import time

import torch
import torch.optim as optim
from torch.utils.data import DataLoader


# Define the Trainer class
class Trainer:
    def __init__(
        self,
        model,
        train_data,
        val_data,
        criterion,
        optimizer,
        batch_size,
        num_epochs,
        save_dir=None,
        save_every=10,
    ):
        self.model = model
        self.train_data = train_data
        self.val_data = val_data
        self.criterion = criterion
        self.optimizer = optimizer
        self.batch_size = batch_size
        self.num_epochs = num_epochs
        # Where to periodically save the model and the copy with the best
        # validation loss. When save_dir is None, no checkpointing happens.
        self.save_dir = save_dir
        self.save_every = save_every
        if save_dir is not None:
            os.makedirs(save_dir, exist_ok=True)
        self.best_val_loss = float("inf")
        self.best_epoch = -1
        # Per-epoch loss history, written to disk after training so a plot can
        # be generated independently of the training run.
        self.history = {"train": [], "val": []}

    def _save(self, path) -> None:
        # Persist the model so it can be loaded later with model.load_state_dict
        torch.save(self.model.state_dict(), path)
        print(f"Saved model -> {path}")

    def train(self) -> None:
        train_loader = DataLoader(
            self.train_data, batch_size=self.batch_size, shuffle=True
        )
        # Validation is optional; some runs may pass val_data=None.
        val_loader = (
            DataLoader(self.val_data, batch_size=self.batch_size, shuffle=False)
            if self.val_data is not None
            else None
        )

        for epoch in range(self.num_epochs):
            self.model.train()
            running_loss = 0.0
            for inputs, targets in train_loader:
                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, targets)
                loss.backward()
                self.optimizer.step()
                running_loss += loss.item()

            train_loss = running_loss / max(len(train_loader), 1)

            # Validation step
            val_loss = float("inf")
            if val_loader is not None:
                self.model.eval()
                val_loss = 0.0
                with torch.no_grad():
                    for inputs, targets in val_loader:
                        outputs = self.model(inputs)
                        loss = self.criterion(outputs, targets)
                        val_loss += loss.item()
                val_loss = val_loss / max(len(val_loader), 1)

            print(
                f"Epoch [{epoch + 1}/{self.num_epochs}], "
                f"Train Loss: {train_loss:.4f}, Validation Loss: {val_loss:.4f}"
            )

            # Record the epoch losses so a loss curve can be plotted later.
            self.history["train"].append(train_loss)
            self.history["val"].append(val_loss)

            # Periodically checkpoint so a long run is not lost and so the latest
            # model is always available under the model's weight folder.
            if self.save_dir is not None:
                if (epoch + 1) % self.save_every == 0:
                    self._save(
                        os.path.join(self.save_dir, f"model_epoch_{epoch + 1:04d}.pt")
                    )
                # Keep the model that achieved the lowest validation loss.
                if val_loss < self.best_val_loss:
                    self.best_val_loss = val_loss
                    self.best_epoch = epoch + 1
                    self._save(os.path.join(self.save_dir, "best_model.pt"))

            # Always keep the latest epoch as a checkpoint when saving is on.
            if self.save_dir is not None and (epoch + 1) == self.num_epochs:
                self._save(os.path.join(self.save_dir, "final_model.pt"))

        if self.save_dir is not None:
            print(
                f"Best validation loss: {self.best_val_loss:.4f} at epoch "
                f'{self.best_epoch if self.best_epoch >= 0 else "?"}.'
            )
            # Persist the per-epoch loss history for plotting (plot_losses.py).
            with open(
                os.path.join(self.save_dir, "loss_history.json"), "w", encoding="utf-8"
            ) as f:
                json.dump(
                    {"train": self.history["train"], "val": self.history["val"]},
                    f,
                    indent=2,
                )
            print(
                f"Loss history saved to "
                f'{os.path.join(self.save_dir, "loss_history.json")}'
            )

    def save_model(self, path) -> None:
        torch.save(self.model.state_dict(), path)
