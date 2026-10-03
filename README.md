# AdvantageKit C++

An unofficial native C++ port of [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) for WPILib 2027. The API and behavior target AdvantageKit `v27.0.0-alpha-6`, with C++-specific adaptations for templates, serialization and native robot lifecycle integration.

This is a personal project. It is not affiliated with or endorsed by Littleton Robotics, Mechanical Advantage, AdvantageKit or my team (254).

## What is included

- Deterministic input logging and replay
- WPILOG readers and writers
- NT4 live publishing
- Logged robot lifecycle and timing
- Driver Station, power distribution, system and radio telemetry
- Replayable NetworkTables inputs and dashboard choosers
- Structured output logging and automatic output registration
- Protobuf logging for types without a struct serializer
- Logged mechanism visualization

## Usage

### Build and publish

Clone the repository with its header-only dependencies and run the release build:

```bash
git clone --recurse-submodules https://github.com/thanewye/advantagekit-cpp.git
cd advantagekit-cpp
./gradlew build -PreleaseMode
```

Create the Maven repository and expanded vendordep JSON with the final version, group and public URLs:

```bash
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=0.1.0-alpha \
  -PpublishGroup=com.example.advantagekit \
  -PvendordepMavenUrl=https://example.github.io/advantagekit-cpp/maven \
  -PvendordepJsonUrl=https://example.github.io/advantagekit-cpp/AdvantageKitCpp-2027.json
```

The Maven repository is written to `build/repos/releases`, and the generated vendordep is written to `build/vendordep/AdvantageKitCpp.json`. See [PUBLISHING.md](PUBLISHING.md) for the complete release process.

### Use in robot code

In VS Code, run **WPILib: Manage Vendor Libraries → Install new libraries (online)** and paste:

```
https://thanewye.github.io/advantagekit-cpp/AdvantageKitCpp-2027.json
```

To pin a release, use `https://thanewye.github.io/advantagekit-cpp/AdvantageKitCpp-<version>.json` instead.

Then inherit the robot class from `akit::LoggedRobot`, configure at least one receiver, and start the logger before constructing logged subsystems:

```cpp
#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/networktables/NT4Publisher.h"

class Robot : public akit::LoggedRobot {
public:
    Robot() {
        akit::Logger::AddDataReceiver(&publisher);
        akit::Logger::Start();
    }

private:
    akit::networktables::NT4Publisher publisher;
};
```

Use `akit::wpilog::WPILOGWriter` for on-robot log storage and `akit::wpilog::WPILOGReader` with `Logger::SetReplaySource()` for replay.

### Migrating from 2026

- WPILib namespaces and headers follow 2027 (`wpi::`, `wpi::math::`, `wpi::units::`, `wpi::nt::`, `.hpp` headers).
- Log timestamps are nanoseconds. Logs written by the 2026 version cannot be replayed.
- Driver Station and system stats keys follow the 2027 opmode, joystick and Systemcore layout.
- `LoggedDashboardChooser` is deprecated in favor of `akit::networktables::LoggedNetworkChooser`, which uses the Selectable dashboard layout.
- `LoggedPowerDistribution::GetInstance` takes a CAN bus as its first argument.
- `ConsoleSource::RoboRIO` is now `ConsoleSource::Systemcore`.

### Template projects

C++ ports of the AdvantageKit template projects are in [`template_projects`](template_projects), with real, sim and replay modes already configured. Copy a directory to start a new robot project. The template projects have not been migrated yet and still target WPILib 2026.

- [`skeleton`](template_projects/skeleton): logger setup only
- [`diff_drive`](template_projects/diff_drive): differential drive with Talon SRX, Talon FX or Spark motors
- [`kitbot_2026`](template_projects/kitbot_2026): 2026 kitbot drive and superstructure
- [`spark_swerve`](template_projects/spark_swerve): swerve with Spark Flex drive, Spark Max turn and high-frequency odometry
- [`talonfx_swerve`](template_projects/talonfx_swerve): swerve configured from Phoenix Tuner X `TunerConstants`
- [`vision`](template_projects/vision): Limelight and PhotonVision pose estimation
