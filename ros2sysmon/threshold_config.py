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