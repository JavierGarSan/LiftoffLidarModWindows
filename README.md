# LiftoffLidar Mod for Windows
Fork of [Jchisholm204/LiftoffSimulator]https://github.com/Jchisholm204/LiftoffSimulator

THIS MOD IS STILL NOT FUNCTIONAL

This is a mod that creates LiDAR data in liftoff and sends it over a UDP port. It uses BepInEx.

- `lf_telemetry` - Pulls base data out of the liftoff simulator

This is a simplified version of Jchisholm204's codebase and I really recommend checking his out. It strips out other parts, such as the virtual controller, and translates the LiDAR mod to windows. 

## Getting Basic Telemetry
Liftoff can export a lot of data over UDP without any modifications.
See this .

Place the following snippet in `~/.config/unity3d/LuGus Studios/Liftoff/TelemetryConfiguration.json`
```json
{ 
	"EndPoint": "127.0.0.1:9001", 
	"StreamFormat": [ 
		"Timestamp", 
		"Position", 
		"Attitude", 
		"Velocity", 
		"Gyro", 
		"Input", 
		"Battery", 
		"MotorRPM" 
	] 
}
```

### Building the Mod
The mod still isn't functional.

### Installing BepInEx
BepInEx is required to load the mod.
It can be downloaded from [https://github.com/bepinex/bepinex/releases](https://github.com/bepinex/bepinex/releases).
Extract the contents of the mod loader into the Liftoff base folder to install it.

#### Configuring BepInEx
Before the mod can be loaded, the BepInEx configuration must be updated.
The configuration file is located under `BepInEx/config/BepInEx.cfg`.

First, disable caching:
```cfg
[Caching]

## Enable/disable assembly metadata cache
## Enabling this will speed up discovery of plugins and patchers by caching the metadata of all types BepInEx discovers.
# Setting type: Boolean
# Default value: true
EnableAssemblyCache = false
```

Then, ensure that the manager object is hidden from Unity:
```cfg
[Chainloader]

## If enabled, hides BepInEx Manager GameObject from Unity.
## This can fix loading issues in some games that attempt to prevent BepInEx from being loaded.
## Use this only if you know what this option means, as it can affect functionality of some older plugins.
## 
# Setting type: Boolean
# Default value: false
HideManagerGameObject = true
```

Finally, set logging to all:
```cfg
[Harmony.Logger]

## Specifies which Harmony log channels to listen to.
## NOTE: IL channel dumps the whole patch methods, use only when needed!
# Setting type: LogChannel
# Default value: Warn, Error
# Acceptable values: None, Info, IL, Warn, Error, Debug, All
# Multiple values can be set at the same time by separating them with , (e.g. Debug, Warning)
LogChannels = All
```
### Installing the Mod
----

