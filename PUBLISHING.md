# Publishing the vendordep

A release consists of immutable Maven artifacts and an `AdvantageKitCpp.json` vendordep whose coordinates and platform list match those artifacts.

## Before the first release

1. Keep the UUID in `AdvantageKitCpp.json` stable across all releases.
2. Keep the WPILib year, dependency versions and CI images aligned.
3. Host the Maven repository and vendordep JSON over anonymous HTTPS.
4. Never overwrite an existing version.

## Build a release candidate

```bash
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=0.1.0-alpha \
  -PpublishGroup=com.ethanye.advantagekit \
  -PvendordepMavenUrl=https://thanewye.github.io/advantagekit-cpp/maven \
  -PvendordepJsonUrl=https://thanewye.github.io/advantagekit-cpp/AdvantageKitCpp.json
```

The Maven repository is written to `build/repos/releases`, the expanded vendordep to `build/vendordep/AdvantageKitCpp.json`, and the platform archives to `build/allOutputs`.

## Validate before release

1. Build every advertised platform through CI.
2. Confirm the header ZIP contains the AdvantageKit, WPILib, Boost.PFR and magic_enum license notices.
3. Install the candidate vendordep into a clean 2026 C++ robot project and compile desktop and roboRIO targets.
4. Run a representative WPILOG replay and compare the expected input/output keys before replacing an existing integration.

## Publish

Run the `Publish` workflow with a new semantic version. It preserves existing Maven versions, updates the stable `AdvantageKitCpp.json`, writes a versioned JSON snapshot, and deploys the combined repository through GitHub Pages.
