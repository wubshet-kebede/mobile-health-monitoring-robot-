#!/bin/bash
# List of models from the AWS Hospital World
MODELS=("XRayMachine" "IVStand" "BloodPressureMonitor" "BPCart" "BMWCart" "CGMClassic" "StorageRack" "Chair" "InstrumentCart1" "Scrubs" "PatientWheelChair" "WhiteChipChair" "TrolleyBed" "SurgicalTrolley" "PotatoChipChair" "VisitorKidSit" "FemaleVisitorSit" "AdjTable" "MopCart3" "MaleVisitorSit" "Drawer" "OfficeChairBlack" "ElderLadyPatient" "ElderMalePatient" "InstrumentCart2" "MetalCabinet" "BedTable" "BedsideTable" "AnesthesiaMachine" "TrolleyBedPatient" "Shower" "SurgicalTrolleyMed" "StorageRackCovered" "KitchenSink" "Toilet" "VendingMachine" "ParkingTrolleyMin" "PatientFSit" "MaleVisitorOnPhone" "FemaleVisitor" "MalePatientBed" "StorageRackCoverOpen" "ParkingTrolleyMax")

echo "Downloading hospital models for Harmonic..."
mkdir -p models

for MODEL in "${MODELS[@]}"
do
    echo "Downloading $MODEL..."
    gz fuel download -u https://fuel.gazebosim.org -m $MODEL -d models -v 4
done
