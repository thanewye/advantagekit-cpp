# AdvantageKit C++

An unofficial native C++ port of [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) for WPILib 2026. The API and behavior target AdvantageKit `v26.0.2`, with C++-specific adaptations for templates, serialization and native robot lifecycle integration.

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
  -PvendordepJsonUrl=https://example.github.io/advantagekit-cpp/AdvantageKitCpp.json
```

The Maven repository is written to `build/repos/releases`, and the generated vendordep is written to `build/vendordep/AdvantageKitCpp.json`. See [PUBLISHING.md](PUBLISHING.md) for the complete release process.

### Use in robot code

Install the published `AdvantageKitCpp.json`, inherit the robot class from `akit::LoggedRobot`, configure at least one receiver, and start the logger before constructing logged subsystems:

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
