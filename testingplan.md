# Testing Plan for ros2sysmon

## Overview
This document outlines the comprehensive testing strategy for the ros2sysmon ROS2 system monitoring application. The system is a multi-threaded Python application using Textual UI framework with external dependencies on ROS2, system calls, and network operations.

## Project Architecture Summary
- **DataCollectionManager**: Coordinates multiple data collectors using threading
- **DisplayManager**: Textual-based terminal UI with real-time updates  
- **SharedDataStore**: Thread-safe data storage with locking mechanisms
- **Collectors**: SystemCollector, NetworkCollector, ROSCollector
- **Configuration**: YAML-based config management with validation

## Testing Framework Setup

### Required Dependencies
Add to package.xml or requirements:
```
pytest>=6.0
pytest-mock>=3.0
pytest-asyncio>=0.18
pytest-cov>=4.0
pytest-timeout>=2.0
```

### Test Directory Structure
```
tests/
├── conftest.py                 # Shared fixtures and setup
├── unit/
│   ├── test_data_models.py
│   ├── test_config_manager.py
│   ├── test_shared_data.py
│   ├── test_display_manager.py
│   └── collectors/
│       ├── test_system_collector.py
│       ├── test_network_collector.py
│       └── test_ros_collector.py
├── integration/
│   ├── test_data_flow.py
│   ├── test_threading.py
│   └── test_configuration_integration.py
├── mocks/
│   ├── test_system_dependencies.py
│   ├── test_ros_dependencies.py
│   └── test_ui_dependencies.py
├── fixtures/
│   ├── sample_configs/
│   └── mock_data/
└── README.md
```

## Test Categories and Priorities

### Priority 1: Critical Foundation Tests

#### SharedDataStore Thread Safety Tests
**Why Critical**: Core multi-threaded architecture component
- Test concurrent read/write operations
- Validate lock acquisition/release behavior
- Test data copying and isolation
- Verify alert queue thread safety
- Test race condition scenarios

#### Configuration Loading and Validation
**Why Critical**: System foundation and startup
- Test valid YAML config loading
- Test malformed config error handling
- Validate threshold boundaries
- Test config defaults and overrides
- Test collection interval validation

#### Data Models Validation
**Why Critical**: Data integrity throughout system
- Test SystemMetrics field validation
- Test SystemAlert level constraints
- Test data class serialization/deserialization
- Validate timestamp handling

### Priority 2: Core Functionality Tests

#### System Collector Tests
**Focus**: Primary monitoring functionality
- Test CPU/memory/disk collection accuracy
- Test temperature sensor fallback behavior
- Test battery detection logic
- Test ROS process filtering
- Test threshold alert generation

#### Mock External Dependencies
**Focus**: Reliable testing environment
- Mock psutil for system metrics
- Mock subprocess for ping/ROS commands
- Mock rclpy for ROS operations
- Test graceful degradation when dependencies fail

#### Integration Data Flow Tests
**Focus**: End-to-end system behavior
- Test collector → SharedDataStore → DisplayManager flow
- Test threading coordination
- Test data update cycles
- Test error propagation

### Priority 3: Robustness and Edge Cases

#### Error Handling Scenarios
- Test network timeouts
- Test missing sensors/permissions
- Test ROS environment failures
- Test UI rendering errors

#### Display Manager Tests
- Test keyboard event handling
- Test mode switching (1/2)
- Test table update logic
- Test alert panel updates

#### Performance and Stability
- Test memory leak detection
- Test performance under load
- Test long-running stability

## Testing Strategies by Component

### Data Models (`data_models.py`)
**Strategy**: Pure unit tests, no mocking needed
- Test dataclass creation and validation
- Test field type checking
- Test timestamp handling
- Test optional field behavior

### Configuration Management
**Strategy**: File-based testing with sample configs
- Create fixture configs (valid/invalid)
- Test YAML parsing edge cases
- Test validation logic
- Test default value application

### SharedDataStore (`shared_data.py`)
**Strategy**: Multi-threaded stress testing
- Use threading.Thread for concurrent access tests
- Test with multiple readers/writers
- Use time.sleep to simulate race conditions
- Verify data consistency after concurrent operations

### System Collector (`collectors/system_collector.py`)
**Strategy**: Mock psutil and subprocess
```python
# Example approach:
@pytest.fixture
def mock_psutil():
    with patch('psutil.cpu_percent') as mock_cpu:
        mock_cpu.return_value = 50.0
        yield mock_cpu

def test_cpu_collection(mock_psutil):
    # Test collector with mocked system data
```

### Network Collector (`collectors/network_collector.py`)
**Strategy**: Mock subprocess ping commands
- Mock successful ping responses
- Mock timeout scenarios
- Mock network unreachable errors
- Test latency calculation

### ROS Collector (`collectors/ros_collector.py`)
**Strategy**: Mock rclpy and ROS CLI commands
- Mock ROS node discovery
- Mock topic list commands
- Mock TF buffer operations
- Test without ROS environment

### Display Manager (`display_manager.py`)
**Strategy**: Mock Textual components
- Mock DataTable widgets
- Mock keyboard events
- Test update cycles without actual UI
- Test CSS class manipulation

## Test Data and Fixtures

### Sample Configuration Files
Create in `tests/fixtures/sample_configs/`:
- `valid_config.yaml` - Complete valid configuration
- `minimal_config.yaml` - Minimal required fields
- `invalid_syntax.yaml` - YAML syntax errors
- `invalid_values.yaml` - Invalid threshold values
- `missing_sections.yaml` - Missing required sections

### Mock Data Sets
Create in `tests/fixtures/mock_data/`:
- `system_metrics.json` - Sample system data
- `ros_nodes.json` - Sample ROS node data
- `topics.json` - Sample topic data
- `tf_frames.json` - Sample TF frame data

### Shared Fixtures (`conftest.py`)
Define reusable fixtures:
- Mock system environment
- Mock ROS environment
- Sample data objects
- Configuration objects
- SharedDataStore instances

## Test Execution Strategy

### Local Development
```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=ros2sysmon --cov-report=html

# Run specific test categories
pytest tests/unit/
pytest tests/integration/
pytest tests/mocks/

# Run tests without ROS dependencies
pytest -m "not requires_ros"
```

### CI/CD Integration
- Use GitHub Actions or similar
- Test matrix: Python 3.8, 3.9, 3.10
- Test with/without ROS2 environment
- Generate coverage reports
- Test on Ubuntu 20.04, 22.04

### Test Markers
Define pytest markers in `pytest.ini`:
```ini
[tool:pytest]
markers =
    slow: marks tests as slow (deselect with '-m "not slow"')
    requires_ros: marks tests that need ROS2 environment
    integration: marks integration tests
    thread_safety: marks thread safety tests
```

## Continuous Testing Guidelines

### Test-Driven Development
1. Write failing test first
2. Implement minimal code to pass
3. Refactor while keeping tests green
4. Focus on one behavior per test

### Code Coverage Goals
- **Unit Tests**: 90%+ coverage
- **Integration Tests**: Cover major data paths
- **Mock Tests**: Cover external dependency failures

### Test Maintenance
- Review and update tests when requirements change
- Remove obsolete tests
- Keep test data current
- Document test intentions clearly

## Mock Strategy Details

### System Dependencies
```python
# Mock psutil for predictable system data
# Mock subprocess for controlled ping responses  
# Mock file system for config loading
# Mock sensors for temperature readings
```

### ROS Dependencies
```python
# Mock rclpy.init/shutdown
# Mock Node creation and destruction
# Mock topic subscription
# Mock TF buffer and listener
# Mock ROS CLI commands (ros2 node list, etc.)
```

### Threading Dependencies
```python
# Use threading.Event for synchronization
# Use time.sleep for race condition simulation
# Use concurrent.futures for parallel execution testing
```

## Success Metrics

### Test Quality Indicators
- All tests pass consistently
- High code coverage (>85%)
- Fast test execution (<30 seconds)
- Clear test failure messages
- No flaky tests

### Regression Prevention
- New features must include tests
- Bug fixes must include regression tests
- Breaking changes must update existing tests
- Performance tests for critical paths

## Next Steps for Implementation

1. **Setup Phase**: Create test directory structure and conftest.py
2. **Foundation Phase**: Implement Priority 1 critical tests
3. **Coverage Phase**: Implement Priority 2 core functionality tests  
4. **Robustness Phase**: Implement Priority 3 edge case tests
5. **Integration Phase**: Setup CI/CD pipeline
6. **Maintenance Phase**: Establish test review and update processes

## Notes for Implementation

- Start with data models - they're pure Python with no dependencies
- SharedDataStore tests are critical due to threading complexity
- Mock external dependencies early to enable isolated testing
- Use fixture data extensively to ensure consistent test conditions
- Document test intentions clearly for future maintainers
- Consider using factory functions for creating test data objects