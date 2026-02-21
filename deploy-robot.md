# Deploy to Robot

## Command

Open a terminal in the robot project folder and run:

```
./gradlew deploy
```

## Steps

1. Connect to the robot's WiFi network (or plug in via USB/Ethernet)
2. Open a terminal in `C:\Users\alyss\2026-Robot-Project`
3. Run the deploy command above
4. Wait for "BUILD SUCCESSFUL" message

## Other Useful Commands

| Command | What it does |
|---|---|
| `./gradlew build` | Compile and run tests (no robot needed) |
| `./gradlew deploy` | Deploy code to the RoboRIO |
| `./gradlew simulateJava` | Run simulation with GUI |
| `./gradlew test` | Run unit tests only |

---

## Finding Device IP Addresses

### RoboRIO
The roboRIO is usually reachable at a predictable address. Try these in order:

- **mDNS hostname:** `10.4.32.2` (team 432 static address) or `roborio-432-frc.local`
- **USB connection:** `172.22.11.2`
- **Ping to confirm:** `ping roborio-432-frc.local`

### Limelight
- **Default hostname:** `limelight.local`
- **Web dashboard:** open `http://limelight.local:5801` in a browser
- **Ping to confirm:** `ping limelight.local`
- If the hostname doesn't resolve, open a browser and scan `10.4.32.1` through `10.4.32.50` — the Limelight web page will load on the right IP.

---

## RoboRIO Diagnostics

### SSH into the RoboRIO
```
ssh admin@10.4.32.2
```
Default password is blank — just press Enter. You can also use `roborio-432-frc.local` instead of the IP.

### Check Disk Space
```bash
df -h
```
Look at the `/` (root) filesystem. If it's over ~80% full, clean up (see below).

### Check What's Using Space
```bash
du -sh /*
du -sh /home/lvuser/*
```

### Clean Up Disk Space

**Delete old deploy logs (most common culprit):**
```bash
rm -rf /home/lvuser/deploy/
```

**Delete old robot logs (Hoot/DataLog files):**
```bash
rm -f /home/lvuser/*.hoot
rm -f /home/lvuser/*.wpilog
```

**Delete old robot code (if redeploying fresh):**
```bash
rm -f /home/lvuser/FRCUserProgram
```

**Check and clear tmp files:**
```bash
rm -rf /tmp/*
```

After cleaning up, re-run `df -h` to confirm space was freed.

### Restart Robot Code Without Rebooting
```bash
killall FRCUserProgram
```
The roboRIO will automatically restart the user program.

### Reboot the RoboRIO
```bash
reboot
```
