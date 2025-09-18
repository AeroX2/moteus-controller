#!/usr/bin/env python3

import os
from pathlib import Path

# PlatformIO script to apply SimpleFOC library patches
Import("env")

def apply_simplefoc_patches():
    """Apply manual patches to SimpleFOC libraries"""
    
    # Get the project and library directories
    project_dir = Path(env["PROJECT_DIR"])
    libdeps_dir = project_dir / ".pio" / "libdeps" / env["PIOENV"]
    
    # Find SimpleFOCDrivers library
    simplefoc_lib = None
    for lib_path in libdeps_dir.glob("*SimpleFOCDrivers*"):
        if lib_path.is_dir():
            simplefoc_lib = lib_path
            break
    
    if not simplefoc_lib:
        print("SimpleFOCDrivers library not found, skipping patches")
        return
    
    print(f"Found SimpleFOCDrivers library at: {simplefoc_lib}")
    
    # Apply STSPIN32G4 pin mapping fix
    apply_stspin32g4_pin_fix(simplefoc_lib)

def apply_stspin32g4_pin_fix(lib_path):
    """Apply the STSPIN32G4 pin mapping fix"""
    
    target_file = lib_path / "src" / "drivers" / "stspin32g4" / "STSPIN32G4.h"
    
    if not target_file.exists():
        print(f"STSPIN32G4.h not found at: {target_file}")
        return
    
    try:
        with open(target_file, 'r') as f:
            content = f.read()
        
        # Check if already patched
        if "STSPIN32G4_PIN_INUL  PE12" in content:
            print("STSPIN32G4 pin mapping patch already applied")
            return
        
        print("Applying STSPIN32G4 pin mapping patch...")
        
        # Apply the pin mapping corrections:
        # Original:  INUL=PE8,  INUH=PE9,  INWL=PE12, INWH=PE13
        # Corrected: INUL=PE12, INUH=PE13, INWL=PE8,  INWH=PE9
        replacements = [
            ("#define STSPIN32G4_PIN_INUL  PE8", "#define STSPIN32G4_PIN_INUL  PE12"),
            ("#define STSPIN32G4_PIN_INUH  PE9", "#define STSPIN32G4_PIN_INUH  PE13"),
            ("#define STSPIN32G4_PIN_INWL  PE12", "#define STSPIN32G4_PIN_INWL  PE8"),
            ("#define STSPIN32G4_PIN_INWH  PE13", "#define STSPIN32G4_PIN_INWH  PE9")
        ]
        
        for old, new in replacements:
            if old in content:
                content = content.replace(old, new)
                print(f"  Replaced: {old} -> {new}")
            else:
                print(f"  Warning: Pattern not found: {old}")
        
        # Write back the patched content
        with open(target_file, 'w') as f:
            f.write(content)
        
        print("STSPIN32G4 pin mapping patch applied successfully")
        
    except Exception as e:
        print(f"Error applying STSPIN32G4 patch: {e}")

# Apply all SimpleFOC patches before build
apply_simplefoc_patches()
