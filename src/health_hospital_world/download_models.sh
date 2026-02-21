#!/bin/bash

MODELS=("XRayMachine" "IVStand" "BloodPressureMonitor" "BPCart" "BMWCart" "CGMClassic" "StorageRack" "Chair" "InstrumentCart1" "Scrubs" "PatientWheelChair" "WhiteChipChair" "TrolleyBed" "SurgicalTrolley" "PotatoChipChair" "VisitorKidSit" "FemaleVisitorSit" "AdjTable" "MopCart3" "MaleVisitorSit" "Drawer" "OfficeChairBlack" "ElderLadyPatient" "ElderMalePatient" "InstrumentCart2" "MetalCabinet" "BedTable" "BedsideTable" "AnesthesiaMachine" "TrolleyBedPatient" "Shower" "SurgicalTrolleyMed" "StorageRackCovered" "KitchenSink" "Toilet" "VendingMachine" "ParkingTrolleyMin" "PatientFSit" "MaleVisitorOnPhone" "FemaleVisitor" "MalePatientBed" "StorageRackCoverOpen" "ParkingTrolleyMax")

echo "Downloading hospital models for Harmonic..."
mkdir -p models

for MODEL in "${MODELS[@]}"
do
    echo "------------------------------------------"
    echo "Downloading: $MODEL"
    # Harmonic requires the full 1.0 API URL to avoid 'Missing Resource' errors
    gz fuel download -v 4 -u "https://fuel.gazebosim.org"
done

# Copy from the hidden Gazebo cache to your local package
echo "Moving models to local package..."
# Note: In Harmonic, the cache path might be ~/.gz/fuel or ~/.ignition/fuel
if [ -d "$HOME/.gz/fuel/models/openrobotics" ]; then
    cp -rn $HOME/.gz/fuel/models/openrobotics/* ./models/
elif [ -d "$HOME/.ignition/fuel/models/openrobotics" ]; then
    cp -rn $HOME/.ignition/fuel/models/openrobotics/* ./models/
fi

echo "Done! Check your models folder now."
