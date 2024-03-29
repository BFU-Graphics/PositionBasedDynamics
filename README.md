# PositionBasedDynamics

[![Cmake Build](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/Ubuntu.yml/badge.svg)](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/Ubuntu.yml)
[![Cmake Build](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/macOS.yml/badge.svg)](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/macOS.yml)
[![Cmake Build](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/Windows.yml/badge.svg)](https://github.com/BFU-Graphics/PositionBasedDynamics/actions/workflows/Windows.yml)

## Introduction

[PositionBasedDynamics](https://github.com/BFU-Graphics/PositionBasedDynamics) is a basic simulation framework to facilitate developing [" Position Based Dynamics "](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf) based projects from scratch.

1. PositionBasedDynamics need no external dependencies but self-contained [libigl](https://github.com/libigl/libigl), which is a simple C++ geometry processing library, and would be automatically included once build.
2. PositionBasedDynamics separates utterly the rendering part and the simulation part on developing.

## Build Instruction

**For all platforms:**

1. Clone [PositionBasedDynamics](https://github.com/BFU-Graphics/PositionBasedDynamics), or simply download this repo.
2. cmake, and build

## Gallery

### Deformable Material

<img src="./Resources/images/cloth_sim01.png"  />

### Rigid Body Dynamics

<img src="./Resources/images/rigidbody_sim02.png"  />

### Fluid Dynamics

comming soon

## Feature

### Cloth

- [x] distance constraint
- [x] dihedral constraint
- [ ] penetration constraint

### Rigid Body

- [x] global BVH collision detection
- [x] rigid-rigid contact constraint
- [ ] rigid-particle contact constraint
