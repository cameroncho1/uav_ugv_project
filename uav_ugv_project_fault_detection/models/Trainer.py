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
        device="cpu",
        writer=None,
        select_by="loss",
    ):
        # Device for training (e.g. 'cpu', 'mps', 'cuda'); batches move with it.
        self.device = torch.device(device)
        self.model = model.to(self.device)
        self.train_data = train_data
        self.val_data = val_data
        self.criterion = criterion.to(self.device)   # moves class weights too
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
        # Optional torch.utils.tensorboard SummaryWriter for live loss curves.
        self.writer = writer
        # 'loss': keep the lowest validation loss as best_model.pt. 'f1': keep the
        # highest validation F1 of the fault class (argmax of classifier logits),
        # which suits rare-fault data where the loss tracks confidence, not detection.
        if select_by not in ("loss", "f1"):
            raise ValueError(f"select_by must be 'loss' or 'f1', not {select_by!r}")
        self.select_by = select_by
        self.best_val_f1 = -1.0

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
            epoch_start = time.time()
            self.model.train()
            running_loss = 0.0
            for inputs, targets in train_loader:
                inputs, targets = inputs.to(self.device), targets.to(self.device)
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
                counts = torch.zeros(3)   # tp, fp, fn of the fault class
                with torch.no_grad():
                    for inputs, targets in val_loader:
                        inputs, targets = inputs.to(self.device), targets.to(self.device)
                        outputs = self.model(inputs)
                        loss = self.criterion(outputs, targets)
                        val_loss += loss.item()
                        if self.select_by == "f1":
                            pred = outputs.argmax(dim=1)
                            counts += torch.stack([
                                ((pred == 1) & (targets == 1)).sum(),
                                ((pred == 1) & (targets == 0)).sum(),
                                ((pred == 0) & (targets == 1)).sum()]).cpu()
                val_loss = val_loss / max(len(val_loader), 1)
                tp, fp, fn = counts.tolist()
                val_f1 = 2 * tp / (2 * tp + fp + fn) if tp else 0.0

            print(
                f"Epoch [{epoch + 1}/{self.num_epochs}], "
                f"Train Loss: {train_loss:.4f}, Validation Loss: {val_loss:.4f}"
                + (f", Validation F1: {val_f1:.4f}"
                   if self.select_by == "f1" and val_loader is not None else "")
            )

            # Record the epoch losses so a loss curve can be plotted later.
            self.history["train"].append(train_loss)
            self.history["val"].append(val_loss)
            if self.writer is not None:
                self.writer.add_scalar("loss/train", train_loss, epoch + 1)
                if val_loader is not None:
                    self.writer.add_scalar("loss/val", val_loss, epoch + 1)
                    if self.select_by == "f1":
                        self.writer.add_scalar("f1/val", val_f1, epoch + 1)
                self.writer.add_scalar(
                    "learning_rate", self.optimizer.param_groups[0]["lr"], epoch + 1)
                self.writer.add_scalar("epoch_seconds", time.time() - epoch_start, epoch + 1)

            # Periodically checkpoint so a long run is not lost and so the latest
            # model is always available under the model's weight folder.
            if self.save_dir is not None:
                if (epoch + 1) % self.save_every == 0:
                    self._save(
                        os.path.join(self.save_dir, f"model_epoch_{epoch + 1:04d}.pt")
                    )
                # Keep the model with the best validation score.
                improved = (val_loader is not None and val_f1 > self.best_val_f1
                            if self.select_by == "f1" else val_loss < self.best_val_loss)
                if improved:
                    self.best_val_loss = val_loss
                    if self.select_by == "f1":
                        self.best_val_f1 = val_f1
                    self.best_epoch = epoch + 1
                    self._save(os.path.join(self.save_dir, "best_model.pt"))

            # Always keep the latest epoch as a checkpoint when saving is on.
            if self.save_dir is not None and (epoch + 1) == self.num_epochs:
                self._save(os.path.join(self.save_dir, "final_model.pt"))

        if self.save_dir is not None:
            score = (f"F1: {self.best_val_f1:.4f}" if self.select_by == "f1"
                     else f"loss: {self.best_val_loss:.4f}")
            print(
                f"Best validation {score} at epoch "
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
