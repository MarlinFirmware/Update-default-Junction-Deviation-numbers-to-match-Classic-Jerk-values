# MMU3 Messages

Starting with the version 2.0.19 of the MMU firmware, requests and responses have a trailing section that contains the CRC8 of the original message. The general structure is as follows:

```
Requests (what Marlin requests):
MMU3:>{RequestMsgCode}{Value}*{CRC8}\n

Responses (what MMU responds with):
MMU3:<{RequestMsgCode}{Value} {ResponseMsgParamCode}{paramValue}*{CRC8}\n
```

An example of that would be:

```
MMU3:>S0*c6\n
MMU3:<S0 A3*22\n

MMU3:>S1*ad\n
MMU3:<S1 A0*34\n

MMU3:>S2*10\n
MMU3:<S2 A2*65\n
```

This set of responses combines to indicate firmware version 3.0.2.

## Startup sequence

When initialized the MMU waits for requests. Marlin repeatedly sends `S0` commands until it gets an answer:
```
MMU3:>S0*c6\n
MMU3:>S0*c6\n
MMU3:>S0*c6\n
...
```

Once communication is established the MMU responds with:
```
MMU3:<S0 A3*22\n
```

Then Marlin continues to get the rest of the MMU firmware version.
```
MMU3:>S1*ad\n
MMU3:<S1 A0*34\n
MMU3:>S2*10\n
MMU3:<S2 A2*65\n
```

Setting the stepper mode to SpreadCycle (M0) or StealthChop (M1):
```
MMU3:>M1*{CRC8};
MMU3:<---nothing---
```

```
MMU3:>P0
MMU3:<P0 A{FINDA status}*{CRC8}\n
```

At this point we can be sure the MMU is available and ready. If there was a timeout or other communication problem somewhere, the printer will not be killed, but for safety the MMU feature will be disabled.

- *Firmware version* is an integer value, and we care about it. As there is no other way of knowing which protocol to use.
- *FINDA status* is 1 if the filament is loaded to the extruder, 0 otherwise.

The *Firmware version* is checked against the required value. If it doesn't match the printer will not be halted, but for safety the MMU feature will be disabled.

## Toolchange

```
MMU3:>T{Filament index}*{CRC8}\n
MMU3:<Q0*ea\n
```

The MMU sends:
```
MMU3:<T{filament index}*P{ProgressCode}{CRC8}\n
```

Which in normal operation would be as follows, let's say that we requested MMU to load `T0``:
```
MMU3:>T0*{CRC8}\n

MMU3:>Q0*{CRC8}\n
MMU3:<T0*P5{CRC8}\n  # P5 => FeedingToFinda

MMU3:>Q0*{CRC8}\n
MMU3:<T0*P7{CRC8}\n  # P7 => FeedingToNozzle
```

As soon as the filament is fed down to the extruder we follow with:

```
MMU3:>C0*{CRC8}\n
```

The MMU will feed a few more millimeters of filament for the extruder gears to grab. When done, the MMU sends:
```
MMU3:>Q0*{CRC8}\n
MMU3:<T0*P9{CRC8}\n  # P9 => FinishingMoves
```

After the `T0*P9` response we immediately continue with the next G-code which should be one or more extruder moves to feed the filament into the hotend.


## FINDA status
```
MMU3:>P0*{CRC8}\n
```

If the filament is loaded to the extruder, FINDA status is 1 and the MMU responds with:
```
MMU3:<P0 A1*{CRC8}\n
```

…otherwise it replies:
```
MMU3:<P0 A0*7b\n
```

This could be used as a filament runout sensor if polled regularly.

## Load filament

To load a filament to the MMU itself, we run:
```
MMU3:>L{Filament index}*{CRC8}\n
MMU3:<L{Filament index} A1*{CRC8}\n
```

…and immediately after that we query the status:
```
MMU3:>Q0*{CRC8}\n
```

The MMU will respond with status messages:
```
MMU3:<L0*P5{CRC8}\n
```

The MMU will load the filament and when done:
```
MMU3:>Q0*{CRC8}\n
MMU3:<L0*P9{CRC8}\n
```

## Unload filament

- `MMU <= 'U0\n'`

The MMU will retract current filament from the extruder, and when done:

- `MMU => 'ok\n'`

## Eject filament

- `MMU <= 'E*Filament index*\n'`
- `MMU => 'ok\n'`
