#!/usr/bin/env sh
rm -f AdvantageKit.json
rm -f ChoreoLib.json
rm -f PathplannerLib.json
rm -f Phoenix6-frc2024-latest.json
rm -f photonlib-json-1.0.json
rm -f REVLib-2024.json
rm -f WPILibNewCommands.json
# additional vendor deps here...
wget https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
wget https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/wpilibNewCommands/WPILibNewCommands.json
wget https://software-metadata.revrobotics.com/REVLib-2024.json
wget https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
wget https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
wget https://sleipnirgroup.github.io/ChoreoLib/dep/ChoreoLib.json
# additional vendor deps here...
