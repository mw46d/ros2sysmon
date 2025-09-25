"""Threshold configuration data structure."""
from dataclasses import dataclass


@dataclass
class ThresholdConfig:
    """System threshold configuration for alerts."""
    cpu_warn: float
    cpu_error: float
    memory_warn: float
    memory_error: float
    disk_warn: float
    disk_error: float
    temperature_warn: float
    temperature_error: float
    network_latency_warn: int
    network_latency_error: int

    def __post_init__(self):
        """Fix invalid values."""
        if self.cpu_warn <= 0 or self.cpu_warn >= 100:
            self.cpu_warn = 40.0
        if self.cpu_error <= 0 or self.cpu_error >= 100:
            self.cpu_error = 50.0
        if self.memory_warn <= 0 or self.memory_warn >= 100:
            self.memory_warn = 40.0
        if self.memory_error <= 0 or self.memory_error >= 100:
            self.memory_error = 50.0
        if self.disk_warn <= 0 or self.disk_warn >= 100:
            self.disk_warn = 40.0
        if self.disk_error <= 0 or self.disk_error >= 100:
            self.disk_error = 50.0
        if self.temperature_warn <= 0 or self.temperature_warn >= 100:
            self.temperature_warn = 40.0
        if self.temperature_error <= 0 or self.temperature_error >= 100:
            self.temperature_error = 50.0
        if self.network_latency_warn <= 0:
            self.network_latency_warn = 40
        if self.network_latency_error <= 0:
            self.network_latency_error = 50