# Mini-Rex FTC 26282 — 2025–2026 Codebase

## Overview

This repository contains the competition code for **FTC Team 26282 Mini-Rex** for the *DECODE (2025–2026)* season.

Our codebase is designed around **modularity, consistency, and reliability**, prioritizing a robot that performs well early and minimizes failure points. The structure emphasizes clean separation between subsystems, controls, and state logic.

---

## Requirements

* Android Studio Ladybug (2024.2) or later
* FTC SDK (included in this repo)

---

## Code Structure

The project follows a structured, subsystem-based architecture:

```
/TeamCode
  /subsystems     → Hardware + logic (drive, intake, etc.)
  /states         → Finite State Machine logic
  /controls       → Driver input handling
  /opmodes        → TeleOp / Auto entry points
```

### Key Design Principles

* **Single-responsibility subsystems**
* **Minimal cross-dependencies**
* **Predictable update loop**
* **Driver-focused reliability over complexity**

---

## Finite State Machine (FSM) System

We use a **custom lightweight FSM structure** to manage robot behavior.

### Why FSM?

* Prevents conflicting commands
* Keeps robot behavior predictable
* Simplifies debugging during matches

### How Ours Works

* Each mechanism operates in a **defined state enum**
* A central update loop runs every cycle
* Transitions are **explicit and controlled**, not automatic

### Example Flow

```
IDLE → INTAKE → HOLD → SCORE → RESET
```

### Implementation Style

* States are **not bloated classes** — kept simple and readable
* Transitions happen through **clear setter methods**
* Subsystems read the current state and act accordingly

### Benefits

* No spaghetti logic in TeleOp
* Easy to tune mid-competition
* Consistent behavior across drivers

---

## Controls

* Designed for **single-driver efficiency**
* Inputs are mapped through a centralized control layer
* Prevents duplicate bindings and accidental conflicts

---

## Getting Started: Cloning

Run this in your terminal:
git clone https://github.com/khan-moazzin/FTC2026.git
```

### Open in Android Studio

* Open → Select project folder
* Sync Gradle
* Deploy to Control Hub / Robot Controller

---

## FTC Resources

* FTC Docs: [https://ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org)
* Javadocs: [https://javadoc.io/doc/org.firstinspires.ftc](https://javadoc.io/doc/org.firstinspires.ftc)
* Community: [https://ftc-community.firstinspires.org](https://ftc-community.firstinspires.org)
