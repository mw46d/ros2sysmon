"""System-level metrics data structure."""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple


@dataclass
class SystemMetrics:
    """System-level metrics like CPU, memory, disk usage."""
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    temperature: Optional[float]
    load_average: Tuple[float, float, float]  # 1min, 5min, 15min
    uptime: str
    battery_percent: Optional[float]
    network_latency: Optional[float]
    timestamp: datetime