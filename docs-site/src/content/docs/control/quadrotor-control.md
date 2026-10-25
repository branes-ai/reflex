---
title: Quadrotor Control Guide
description: How quadrotor control changes across six speed strata from 15 to 720 km/h, why the vehicles in each stratum look the way they do, and the control problems reflex must design, implement, and test for.
---

This guide covers quadrotor flight control from the rotors up. It does three
things:

1. It characterizes six **performance strata**, from 15 km/h indoor research
   vehicles to 720 km/h speed-record airframes. For each it gives the size,
   weight, motors, and thrust, and explains why those parameters follow from
   the speed.
2. It derives what changes for the **controller** between strata: which
   physical effects dominate, how fast the loops must run, and which model
   terms the controller and the simulator need.
3. It lists the **control problems** reflex must solve, with a design question,
   an implementation home, and a test plan for each, and maps every problem
   onto the strata where it matters.

It sits above the component pages ([PID](/control/pid/),
[quadrotor model](/model/quadrotor/)) and links the roadmap issues that carry
the work.

## Conventions

The conventions are those of the [quadrotor model](/model/quadrotor/):

- **Frames:** the world frame has z up. The body frame is x forward, y left,
  z up (FLU).
- **Rotor order:** PX4 Quad X. Rotor 0 is front-right CCW, rotor 1 rear-left
  CCW, rotor 2 front-left CW, rotor 3 rear-right CW.
- **Symbols:**
  - $n = T_{max}/(m g)$ is the thrust-to-weight ratio.
  - $q = \tfrac12 \rho v^2$ is the dynamic pressure, with
    $\rho = 1.225\ \mathrm{kg/m^3}$ at sea level.
  - $C_D A$ is the drag area.
  - $J = V/(n_{rev} D)$ is the propeller advance ratio: airspeed $V$ along the
    rotor axis, rotor speed $n_{rev}$ in rev/s, diameter $D$.

## The control stack in one page

```
 planner / MPC (companion, 100–200 Hz)
        │  collective thrust + body rates (CTBR), optional feed-forward torques
        ▼
 attitude loop  (FC, 250–1000 Hz)      quaternion error → rate setpoint
        ▼
 rate loop      (FC, 1–8 kHz)          PID or INDI on filtered gyro → torque demand
        ▼
 mixer          (FC, same rate)        wrench → four rotor thrusts, saturation priority
        ▼
 motor command  (ESC, DShot)           thrust → rotor speed / throttle, battery compensation
        ▼
 rotors + airframe  ← modelled by QuadrotorModel in SITL
```

reflex owns everything from the attitude loop down, plus the vehicle model.
The planner and the state estimate come from cortex and racing-drone. Across
the six strata the stack keeps the same shape. What changes is the physics
inside each box, the rates the loops must run at, and which saturations bind.

## Performance strata

A stratum is named by a nominal speed near the top of its vehicle class's
operating envelope: the speed the class races or sprints at in normal use,
not a one-off record run. Cruise speeds are lower; a camera drone, for
example, is rated for endurance well below its top speed [4]. Each stratum
has reference vehicles.
Manufacturer and record figures are cited. Values marked **≈** are this
guide's engineering estimates, computed from the cited figures with the
relations in [why the parameters are what they are](#why-the-parameters-are-what-they-are).

### Summary

| Stratum | Speed | Reference vehicle | All-up mass | Size | Props | Motors | Power system | Thrust / weight |
|---|---|---|---|---|---|---|---|---|
| **S0 ultra-slow** | 15 km/h (4 m/s) | Crazyflie 2.1 / 2.1 Brushless (research nano) | 27 g [1]; 34 g brushless [2] | 92 mm motor-to-motor [1] | 45–55 mm, 2-blade | 7×16 mm coreless brushed [1]; 08028 10000 KV brushless [2] | 1S LiPo, 7–10 min [1, 2] | ≈ 2 brushed; ≈ 3.5 brushless (4 × 30 g [2] on 34 g) |
| **S1 slow-slow** | 30 km/h (8 m/s) | 65 mm brushless tiny whoop (BetaFPV Meteor65) | ≈ 31 g: 23 g dry [3] + 1S 300 mAh pack | 65 mm motor diagonal, ducted | 31–35 mm, 3-blade [3] | 0802, 19 500–22 000 KV, 1.8 g each [3] | 1S 300 mAh, ≈ 4.5 min [3] | ≈ 3–4 |
| **S2 slow** | 60 km/h (17 m/s) | Consumer camera drone (DJI Mini 4 Pro: 57.6 km/h max [4]) | < 249 g [4] | ≈ 250 mm diagonal, folding | ≈ 6", 2-blade, low pitch | small outrunners, efficiency-wound | 2S Li-ion, ≈ 30 min | ≈ 2 |
| **S3 medium** | 120 km/h (33 m/s) | 5"–6" racing quad (MonoRace A2RL 2025 drone; DCL racer) | 966 g [5]; DCL rule 850–1000 g [6] | ≥ 360 mm motor diagonal (DCL) [6] | 5.1", 3-blade [5] | 2207-class, 1750–1950 KV, ≈ 1.7–1.9 kg static thrust each [7] | 6S LiPo, ≈ 2 min | ≈ 7 (MonoRace manoeuvres reach ≈ 7 g [5]) |
| **S4 fast** | 360 km/h (100 m/s) | Red Bull camera drone (filmed F1 at speed) | < 1 kg [8] | streamlined fuselage | high pitch; rotors up to 42 000 rpm [8] | high-KV racing motors, partly unreleased [8] | LiPo, ≈ 3 min [8] | ≈ 10+ (100 → 300 km/h in ≈ 2–4 s [8]) |
| **S5 ultra (torpedo)** | 720 km/h (200 m/s) | Speed-record quads: Peregreen 2 / 3 / V4 | 2.77 kg (Peregreen 3) [9] | single-piece streamlined body, canted rotors | 6" (V4) [10], very high pitch | T-Motor 3115 900 KV (P2) [11]; 3120 900 KV (V4) [10] | 12S LiPo, 120 A ESCs (P2) [11]; ≈ 16 kW (P3) [9]; < 2 min | ≈ 10–15 |

**720 km/h is beyond the current record.** Peregreen V4 holds the Guinness
quadcopter record at 657.59 km/h, set on 11 December 2025 [10, 12]. Peregreen
3 flew 585 km/h on about 16 kW [9]. S5 is therefore a projection: holding the
Peregreen 3 drag area, 200 m/s needs about **30 kW** of electrical power
([power ceiling](#3-the-power-ceiling)). The speed column for S5 is a
design target, not an observed operating point.

### S0 — ultra-slow, 15 km/h

**Use.** Indoor research, swarms, flight near people, and learning-based
control experiments.

**Why it looks like this.**
- The vehicle is light because low mass keeps collision energy small and
  crashes cheap, not because of speed. Light is not harmless: spinning props
  still cut, so flight near people still needs prop guards and separation.
- Its speed is set by the mission and the firmware, not by physics. A 3.5
  thrust-to-weight brushless Crazyflie could fly much faster. Indoors,
  15 km/h already covers a room in two seconds, and the motion-capture or
  flow-deck estimate degrades with speed.

**Control character.**
- The dynamics are hover-dominated, so small-angle linearization works.
  Dynamic pressure is about 11 Pa, so aerodynamic drag is a small disturbance.
- Rotor dynamics are slow relative to the airframe. Coreless brushed motors
  have a time constant of about 65–72 ms [13], against 20–25 ms for racing
  motors [5]. The rate loop must therefore respect motor lag explicitly.
- The dominant disturbances are ground effect, ceiling effect, and downwash
  between vehicles in a swarm.
- Thrust is very sensitive to battery voltage on 1S, so voltage
  compensation is essential [14].

### S1 — slow-slow, 30 km/h

**Use.** Indoor FPV, proximity inspection, and confined-space flight.

**Why it looks like this.**
- Ducts protect the props and add a little static thrust, at the cost of
  drag.
- Very high KV on 1S spins tiny props fast enough for a thrust-to-weight
  ratio of 3–4.

**Control character.**
- The airframe is the most agile of all strata in angular acceleration,
  because angular acceleration scales as $1/L^2$
  ([scaling](#4-scaling-with-size)). The rate loop needs high bandwidth.
- Gyro noise and frame resonances sit close to the control band, so
  filtering (reflex#21) is critical.
- Wall and duct aerodynamics matter.

### S2 — slow, 60 km/h

**Use.** Aerial photography, mapping, delivery, and inspection.

**Why it looks like this.**
- The vehicle is optimized for **endurance**, not agility. It uses large,
  low-pitch, low-disc-loading props, low-KV efficient motors, Li-ion cells,
  and a thrust-to-weight ratio near 2.
- The class tops out near the stratum speed (16 m/s, 57.6 km/h, in sport
  mode [4]), enough to fight a 10 m/s wind with margin. It cruises far
  slower: DJI rates the maximum flight time at 21.6 km/h [4].
- Regulation drives the mass: 249 g is the threshold for lighter rules in
  many jurisdictions [4].

**Control character.**
- The controller is tuned for smoothness and disturbance rejection: gimbal
  stability, wind gusts, and position hold on GNSS.
- Attitude excursions are small (< 35°).
- The binding constraint is thrust margin. At $n \approx 2$ a gust plus a
  climb demand can saturate the motors, so the mixer's thrust priority
  matters (reflex#19).

### S3 — medium, 120 km/h

**Use.** Drone racing, human and autonomous. This is the stratum
racing-drone targets for A2RL.

**Why it looks like this.**
- At $n \approx 7$ the vehicle accelerates at up to 7 g and can hold about
  80° of tilt at full thrust. That is what gate-to-gate racing needs, with
  turns of 20–30 m radius at 30 m/s.
- The 5.1" props and 2207 motors on 6S are a mature sweet spot. Static thrust
  is high, the props survive crashes, and the pitch speed (about 60 m/s for a
  4.1" pitch at about 34 000 rpm loaded, ≈) sits above the operating speed
  with useful margin.
- Mass is held near 1 kg by league rules [6] and carries the compute (a
  Jetson-class companion on A2RL drones).

**Control character.**
- This is the first stratum where **aerodynamics shape the trajectory**:
  - Dynamic pressure reaches about 680 Pa.
  - Rotor drag, which is proportional to velocity, and body drag change the
    achievable acceleration. MonoRace identifies linear and quadratic drag
    terms for its EKF and policy [5].
  - Thrust lapses with advance ratio: the same command gives less thrust at
    30 m/s than in hover.
- **Saturation is the normal operating condition**, not an exception.
  Time-optimal flight rides the thrust limit, so mixer priority and
  integrator anti-windup decide whether the vehicle keeps attitude
  authority.
- **IMU saturation.** MonoRace hit the 16 g accelerometer limit and had to
  fall back to the model [5].
- **Latency budget.** 10 ms is 0.33 m of travel. The racing-drone end-to-end
  budget is ≤ 29 ms.

### S4 — fast, 360 km/h

**Use.** Chase filming of race cars and motorcycles, and high-speed
interception.

**Why it looks like this.**
- To reach 100 m/s the airframe stops being a frame with props and becomes a
  fuselage. Dynamic pressure is about 6 kPa, nine times S3, so every
  square centimetre of frontal area counts.
- At 42 000 rpm (700 rev/s) [8] the propeller needs a pitch of at least
  5.5" just to reach the zero-thrust advance ratio at 100 m/s. To still
  produce thrust there it needs about 7" of pitch or more, which means a
  pitch-to-diameter ratio well above 1, unlike the 0.8 of a racing prop (≈).
- High-pitch props give poor static thrust per watt, so the vehicle is a
  compromise between launch acceleration and top speed.

**Control character.**
- The vehicle flies at **high tilt** (≈ 70–85°) for long periods, and its
  envelope reaches toward 90°, so attitude representation is no longer
  optional. Euler angles are singular at 90° pitch, and the controller must
  work on $SO(3)$ or with quaternions.
- The body produces aerodynamic lift and moments that the rotors must trim.
  Gains that suit hover are wrong at 6 kPa, so **gain scheduling with dynamic
  pressure** (or with airspeed) is needed.
- Turn radius at 5 g is about 200 m: the vehicle flies lines and arcs, not
  gates.
- Estimation is GNSS-aided at high speed, and 10 ms of latency is 1 m of
  travel.

### S5 — ultra / torpedo, 720 km/h

**Use.** Speed records and research into the limits of electric
multirotor flight.

**Why it looks like this.** Every design choice serves top speed in a
straight line:
- The rotors are canted forward relative to a torpedo-shaped body, so the
  body is aligned with the airflow at the cruise tilt. Here $\theta$ is the
  tilt of the rotor thrust from vertical; because of the cant, the body
  attitude differs from it.
  - At the drag–thrust limit, full thrust gives
    $\theta_{max} = \arccos(1/n)$, roughly 84° for $n = 10$.
  - That is a limit, not the cruise point. With the drag area estimated in
    the [power ceiling](#3-the-power-ceiling) and Peregreen 3's 2.77 kg, drag
    at 200 m/s is about 76 N against 27 N of weight, so the rotor-thrust tilt
    is $\arctan(D/mg) \approx 70°$ before body lift (≈). S5 is limited by
    power and tip Mach, not by the drag–thrust balance.
- Props are small (6" on V4 [10]) with very high pitch, on 900 KV motors at
  high voltage.
- Batteries are chosen for power density, not energy. Peregreen 3 draws
  about 16 kW for under two minutes and needed water cooling after a
  prototype caught fire [9].
- Mass goes *up*, to 2.77 kg on Peregreen 3: the extra inertia improves
  stability at speed [9], and the power system is heavy.

**Control character.**
- **Transonic propeller tips.** A 6" prop at 0.85 × the 12S no-load speed of
  a 900 KV motor (≈ 640 rev/s) has a rotational tip speed of about 310 m/s.
  Combined with 200 m/s of forward flight, the helical tip speed is about
  370 m/s, Mach 1.08 (≈).
  - Thrust and torque coefficients change abruptly near Mach 1, so
    $T = k_f \omega^2$ no longer holds even approximately.
  - Holding the helical tip below Mach 0.9 at 200 m/s caps the rotational tip
    speed near 235 m/s. Reaching 240 m/s of pitch speed then needs a
    pitch-to-diameter ratio above 3 (≈).
  - This is the physical wall that makes 720 km/h harder than the current
    record.
- **The thrust map is a function of airspeed.** A
  [$C_T(J)$](#e-aerodynamic-model-terms) thrust model is mandatory in both
  the controller and the simulator.
- **Control authority shrinks at speed.** The motors are near their power
  limit, so the differential thrust left for attitude is small. Mixer
  saturation priority is a flight-critical feature.
- **Structure and thermal.** Aeroelastic flutter, ESC and motor thermal
  limits, and battery sag over a two-minute run are part of the control
  envelope.
- **Very large turn radius.** At 5 g the radius is about 820 m, and 10 ms of
  latency is 2 m of travel.

## Why the parameters are what they are

Four ceilings bound a quadrotor's speed. Each stratum is limited by a
different one, and the vehicle parameters follow from which ceiling binds.

### 1. The drag–thrust balance

In steady level flight at tilt $\theta$, thrust balances weight vertically and
drag horizontally:

$$
T\cos\theta = m g, \qquad T\sin\theta = \tfrac12 \rho\, C_D A\, v^2
\;\;\Rightarrow\;\;
v_{max} = \sqrt{\frac{2\, m g \sqrt{n^2 - 1}}{\rho\, C_D A}} .
$$

Speed grows only as $\sqrt{n}$ and as $1/\sqrt{C_D A}$. Doubling the speed
needs four times $n$, or a quarter of the drag area. This is why S4 and S5
spend their design effort on the fuselage. For the small strata this ceiling
lies far above the operating speed, which the mission sets instead.

### 2. The propeller pitch-speed ceiling

A fixed-pitch propeller's thrust falls as the axial inflow approaches its
pitch speed $V_p \approx P \cdot n_{rev}$, and reaches zero near the
zero-thrust advance ratio $J_0$ (a little below $P/D$ for small props). To
fly at speed $v$:

$$
P \gtrsim \frac{v}{n_{rev}} \quad\text{(plus slip margin)}.
$$

- **S3:** about 60 m/s of pitch speed limits a racing quad to the
  140–160 km/h region quoted for racers [6, 15]. That ceiling, not the body
  drag, is what binds.
- **S4 and S5:** reaching these speeds needs high pitch-to-diameter ratios,
  and the price is poor static thrust per watt.

### 3. The power ceiling

The electrical power to overcome body drag is
$P_{el} = \tfrac12 \rho\, C_D A\, v^3 / \eta$, where
$\eta = \eta_{ESC}\,\eta_{motor}\,\eta_{prop}$ is the overall
electrical-to-thrust-power efficiency.

- **Anchoring the drag area.** The 16.2 kW reported for Peregreen 3 [9] is
  taken as electrical draw at 162.5 m/s. With $\eta \approx 0.5$ (about
  0.85 for ESC and motor times 0.6 for the propeller) it implies
  $C_D A \approx 0.0031\ \mathrm{m^2}$, about a 9 × 9 cm frontal area at
  $C_D \approx 0.37$ (≈).
- **At 200 m/s** the same body and efficiency need about 30 kW of electrical
  power: 1.86 times the power for 1.23 times the speed (≈). The ratio holds
  for any $\eta$ that stays constant; the drag area itself scales with the
  assumed $\eta$.
- **Consequences.** The battery and ESCs are sized for peak power, the
  vehicle is heavier, and flight time is minutes.

### 4. Scaling with size

Scale the airframe geometrically by a length $L$ and hold the propeller tip
speed (a Mach limit), so $n_{rev} \propto 1/D$:

| Quantity | Scaling | Consequence |
|---|---|---|
| Mass $m$ | $L^3$ | — |
| Thrust $\propto \rho n_{rev}^2 D^4$ | $L^2$ | $n \propto 1/L$: small vehicles have spare thrust |
| Inertia $J$ | $L^5$ | — |
| Torque $\propto T L$ | $L^3$ | angular acceleration $\propto 1/L^2$: small vehicles flip fastest |
| Natural frequencies | $\propto 1/L$ | small vehicles need faster loops and better filtering |

This is why S0 and S1 are the most agile in rotation yet the slowest in
translation. Their speed is set by use and by drag on a bluff body, not by
thrust. It is also why S5 grows heavier: the inertia damps disturbances at
speed, and the motors that deliver 16 kW have mass of their own.

### Per-stratum physics at a glance

| | S0 | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Speed [m/s] | 4.2 | 8.3 | 16.7 | 33.3 | 100 | 200 |
| Dynamic pressure $q$ [Pa] | 11 | 43 | 170 | 680 | 6 100 | 24 500 |
| Travel in 10 ms | 4 cm | 8 cm | 17 cm | 33 cm | 1.0 m | 2.0 m |
| Turn radius at 5 g | 0.4 m | 1.4 m | 5.7 m | 23 m | 200 m | 820 m |
| Typical rotor-thrust tilt | < 20° | < 45° | < 35° | up to 80° | ≈ 70–85° | ≈ 70° at 200 m/s, before body lift |
| Binding ceiling | mission | mission / drag | mission / endurance | pitch speed, thrust | drag, pitch | power, tip Mach |
| Is $T = k_f\omega^2$ adequate? | yes | yes | yes, with care | no: rotor drag, $C_T(J)$ | no | no: transonic |

## What the current reflex model covers

[`QuadrotorModel`](/model/quadrotor/) implements:

- rigid-body 6-DOF dynamics;
- first-order motor lag;
- the static thrust law $T = k_f\omega^2$ and the yaw-torque law $k_m T$;
- linear and quadratic body drag;
- battery sag;
- ground contact.

That set is adequate for **S0–S2 in free flight away from surfaces**, and
for S3 at low speed. It has ground contact but no ground or ceiling effect
(E4), which is critical for S0 and S1, so near-surface scenarios in those
strata are *model-limited* until E4 lands. Above about
15 m/s the effects listed under [aerodynamic model terms](#e-aerodynamic-model-terms)
become first-order, and SITL results there must be read as optimistic until
they are modelled. Each term is a separate model feature with its own verification and
validation pass.

## Control problems to design, implement, and test

Each problem below lists:
- **Design:** the question that must be answered before code. Stateful types
  are specified model, invariants, and contracts first.
- **Implement:** where the code lives.
- **Test:** the evidence that closes it.
- **Strata:** where it matters (● critical, ○ relevant, – negligible).

### A. Allocation and actuation

**A1. Mixer and saturation priority** — reflex#19.
- **Design:**
  - Closed-form inverse of the X allocation matrix.
  - Priority order: roll/pitch, then collective thrust, then yaw.
  - Two-sided thrust shift (airmode).
  - Saturation report for anti-windup.
- **Implement:** `control/mixer.hpp`.
- **Test:**
  - Round trip inside the limits.
  - Corner cases of the thrust envelope.
  - Cross-check of the torque signs against `QuadrotorModel`.
- **Strata:** S0 ○ · S1 ○ · S2 ● · S3 ● · S4 ● · S5 ●

**A2. Thrust linearization and battery compensation.**
- **Design:** command → thrust map, with thrust ∝ throttle² and a
  voltage-sag correction, as in Bitcraze's thrust compensation [14].
- **Implement:** a motor-output stage after the mixer.
- **Test:** SITL with battery sag enabled. The hover throttle must stay
  constant over a full pack within ±3 %.
- **Strata:** S0 ● · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

**A3. Motor dynamics compensation.**
- **Design:** feed-forward lead $1 + \tau s$ on the thrust command, or INDI
  with rotor-speed feedback (DShot telemetry).
- **Implement:** motor-output stage, or the INDI rate loop (B3).
- **Test:**
  - SITL step response with $\tau$ in the range 20–70 ms.
  - Phase margin measured, not assumed.
- **Strata:** S0 ● · S1 ○ · S2 ○ · S3 ● · S4 ● · S5 ●

**A4. Motor-loss detection and three-rotor fallback** (later phase).
- **Design:** detect a lost rotor from the residual between commanded and
  measured rotor speed. Then give up yaw control: the vehicle spins about its
  yaw axis and controls its reduced attitude.
- **Implement:** a mixer mode plus an attitude-loop mode.
- **Test:** SITL fault injection (racing-drone#16 adds fault injection).
- **Strata:** S0 – · S1 – · S2 ● · S3 ○ · S4 ○ · S5 –

### B. Rate loop

**B1. Cascaded PID rate loop** — reflex#20.
- **Design:**
  - D on measurement, with a D-term low-pass filter.
  - Integrator anti-windup driven by the mixer's saturation report (A1).
  - Feed-forward from the setpoint derivative.
  - "I-term relax" during fast stick or setpoint moves.
- **Implement:** `control/pid.hpp` (exists) and `control/rate_controller.hpp`.
- **Test:**
  - Bandwidth and overshoot specifications per stratum parameter set.
  - No integrator windup through a 2 s saturation.
- **Strata:** ● in every stratum.

**B2. Gyro filtering** — reflex#21.
- **Design:**
  - Low-pass filter plus static and dynamic notch filters.
  - RPM-tracked notches at the rotor harmonics.
  - The filter-delay budget is part of the phase margin.
- **Implement:** `dsp/`.
- **Test:**
  - Group delay at the crossover frequency below a specified limit.
  - Attenuation at the rotor frequency.
  - SITL with injected vibration.
- **Strata:** S0 ○ · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

**B3. INDI (incremental nonlinear dynamic inversion)** as an alternative rate
loop.
- **Design:**
  - Angular-acceleration estimate from the filtered gyro.
  - Control effectiveness from the mixer matrix.
  - Rotor-speed feedback.
  - Robust to model error because it acts incrementally.
- **Implement:** `control/indi.hpp`.
- **Test:**
  - Disturbance rejection against the PID baseline in SITL.
  - Sensitivity to errors in the control-effectiveness estimate.
- **Strata:** S0 – · S1 ○ · S2 ○ · S3 ● · S4 ● · S5 ●

**B4. Gain scheduling.**
- **Design:**
  - Throttle-PID attenuation (TPA) at high collective thrust.
  - At speed, schedule on dynamic pressure $q$, with the estimate of $q$
    coming from cortex.
  - Gains are fixed per schedule point: no reconfiguration at runtime.
- **Implement:** a schedule table in the rate controller.
- **Test:**
  - Stability margins at every schedule point.
  - Bumpless transfer between points.
- **Strata:** S0 – · S1 – · S2 ○ · S3 ● · S4 ● · S5 ●

### C. Attitude loop

**C1. Quaternion attitude control on $SO(3)$** — reflex#20.
- **Design:**
  - No Euler angles anywhere in the control path.
  - Error from $q_{err} = q^{-1} \otimes q_{sp}$, taking the shortest-path
    sign.
- **Implement:** `control/attitude_controller.hpp`.
- **Test:**
  - Large-angle steps of 180° in roll and pitch.
  - Hold at 85° tilt, as an envelope stress case beyond the S5 cruise tilt.
  - Continuity of the error through the antipodal quaternion.
- **Strata:** S0 ○ · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

**C2. Tilt-prioritized (reduced-attitude) control.**
- **Design:**
  - Split the error into tilt and yaw.
  - Track the tilt first, because it carries the thrust vector.
  - Yaw has lower bandwidth and yields under saturation, consistent with
    the mixer priority (A1).
- **Implement:** the attitude controller.
- **Test:**
  - Combined tilt and yaw steps.
  - The tilt error converges regardless of the yaw error.
- **Strata:** S0 ○ · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

### D. Translational control and the planner interface

**D1. CTBR command interface.**
- **Design:**
  - Collective thrust plus body rates, with optional torque feed-forward.
  - The command carries a timestamp, a validity window, and staleness
    handling.
- **Implement:** `control/` message types, mirrored in racing-drone's
  `msg/`.
- **Test:**
  - Staleness: a command older than the window is replaced by the safe
    fallback.
  - The latency is measured end to end.
- **Strata:** S0 ○ · S1 ○ · S2 ○ · S3 ● · S4 ● · S5 ●

**D2. Velocity and position loops** — reflex#20.
- **Design:**
  - Acceleration command → thrust vector → attitude setpoint.
  - Feed-forward from differential flatness.
  - Drag compensation.
  - Thrust-vector saturation that keeps the vertical component first.
- **Implement:** `control/position_controller.hpp`.
- **Test:**
  - Racing-drone gate P1: ≥ 99 % of 500 seeds at ≤ 5 m/s over two laps.
  - Tracking RMS per stratum.
- **Strata:** S0 ● · S1 ● · S2 ● · S3 ● · S4 ○ · S5 ○

**D3. MPC (MPCC) as the planner.**
- **Design:** RTI-SQP, with the model in E as the prediction model.
- **Implement:** reflex roadmap Phase 3.
- **Test:** racing-drone gates P2–P3.
- **Strata:** S0 – · S1 – · S2 – · S3 ● · S4 ○ · S5 –

### E. Aerodynamic model terms

These are needed in `QuadrotorModel` for SITL fidelity, and in the controller
for compensation.

**E1. Rotor drag (blade flapping and H-force).**
- **Design:** a force proportional to the in-plane velocity, scaled by
  thrust: $-k_d\, T\, v_{\perp}$.
- **Test:** compare the predicted acceleration against flight logs, using
  MonoRace's identified terms [5] as a reference.
- **Strata:** S0 – · S1 ○ · S2 ○ · S3 ● · S4 ● · S5 ●

**E2. Thrust lapse $C_T(J)$.**
- **Design:** a thrust coefficient that depends on the axial advance ratio,
  from propeller data or a blade-element model.
- **Test:** thrust-stand data or a wind-tunnel table.
- **Strata:** S0 – · S1 – · S2 ○ · S3 ● · S4 ● · S5 ●

**E3. Body lift and moments.**
- **Design:** lift, drag, and pitching moment as functions of the angle of
  attack, from AirShaper-style CFD or a wind tunnel.
- **Test:** CFD tables.
- **Strata:** S0 – · S1 – · S2 – · S3 ○ · S4 ● · S5 ●

**E4. Ground and ceiling effect.**
- **Design:** thrust gain as a function of height over rotor radius.
- **Test:** a hover-height sweep.
- **Strata:** S0 ● · S1 ● · S2 ○ · S3 ○ · S4 – · S5 –

**E5. Vortex ring state in fast descent.**
- **Design:** detect it, and constrain the vertical descent rate.
- **Test:** an SITL descent-rate sweep.
- **Strata:** S0 ○ · S1 ○ · S2 ● · S3 ○ · S4 – · S5 –

**E6. Compressibility at the prop tips.**
- **Design:** correct $C_T$ and $C_Q$ for the helical tip Mach number.
- **Test:** propeller data.
- **Strata:** S0 – · S1 – · S2 – · S3 – · S4 ○ · S5 ●

### F. Interfaces to estimation

**F1. IMU saturation and model fallback.**
- **Design:**
  - Flag accelerometer and gyro saturation.
  - When saturated, the controller uses the model-predicted acceleration,
    as MonoRace does [5].
  - Shared with cortex.
- **Test:** SITL with sensor clipping injected.
- **Strata:** S0 – · S1 ○ · S2 – · S3 ● · S4 ● · S5 ●

**F2. Latency compensation.**
- **Design:**
  - Forward-propagate the estimate to the actuation time with IMU
    propagation (reflex#13 roadmap).
  - The budget is stated per stratum.
- **Test:** delay injection in SITL. The tracking error must stay within
  specification at 1.5× the nominal delay.
- **Strata:** S0 ○ · S1 ○ · S2 ○ · S3 ● · S4 ● · S5 ●

**F3. Vibration aliasing.**
- **Design:**
  - Choose the IMU sample rate and anti-alias filtering against the rotor
    frequencies.
  - Notch frequencies follow the RPM (B2).
- **Test:** spectral tests on logs.
- **Strata:** S0 ○ · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

### G. Power, thermal, and structure

**G1. Battery and current limits.**
- **Design:**
  - Make the thrust envelope a function of state of charge and current
    limit.
  - The mixer's maximum thrust comes from the power stage, not a constant.
- **Test:** an SITL full-pack run with sag.
- **Strata:** S0 ● · S1 ● · S2 ○ · S3 ● · S4 ● · S5 ●

**G2. Thermal derating.**
- **Design:** derate the ESC and motor current limits with temperature, and
  model the temperature itself.
- **Test:** thermal-model SITL, then a bench test.
- **Strata:** S0 – · S1 – · S2 – · S3 ○ · S4 ● · S5 ●

**G3. Aeroelastic and structural modes.**
- **Design:** frame resonances and flutter speed; the structure must keep a
  margin above the control bandwidth.
- **Test:** modal survey and notch placement.
- **Strata:** S0 – · S1 ○ · S2 – · S3 ○ · S4 ● · S5 ●

### H. Safety and lifecycle

**H1. Arming, disarming, and idle.**
- **Design:**
  - The airmode thrust shift (A1) is active only when armed.
  - Motor idle is set on the ground.
  - Disarm on landing detection.
- **Implement:** the FC state machine.
- **Test:**
  - State-machine model checking.
  - No motor output while disarmed.
- **Strata:** ● in every stratum.

**H2. Failsafes.**
- **Design:**
  - Link loss → hover, land, or kill, depending on the stratum.
  - Stale-command handling (D1).
  - Estimator-health gating with NIS (reflex#13).
- **Test:** fault-injection scenarios.
- **Strata:** ● in every stratum.

**H3. Envelope protection.**
- **Design:** limits on tilt, body rate, and speed that are configurable per
  stratum, plus a geofence.
- **Test:** SITL scenarios commanded beyond the envelope.
- **Strata:** S0 ● · S1 ● · S2 ● · S3 ○ · S4 ● · S5 ●

## Test strategy across strata

Strata make a natural **test matrix**. The proposal:

1. **One vehicle parameter set per stratum**, as racing-drone `vehicles/*.yaml`
   files with reflex test fixtures that mirror them.
   - S3 is calibrated from the MonoRace and RP-1 numbers.
   - The other strata start from the reference values in the
     [summary table](#summary) and are labelled as estimates until measured.
2. **Component tests run across the sweep.** The mixer, rate, and attitude
   tests are parameterized over S0–S5, in the same way that `Pid` is
   parameterized over scalar types.
   - Specifications are written in normalized form: bandwidth as a fraction
     of the motor bandwidth $1/\tau$, overshoot in percent, settling time in
     multiples of $\tau$.
   - One test can then judge every stratum.
3. **SITL scenarios per stratum.**
   - Hover and position hold for all strata.
   - Gate courses for S3.
   - Straight-line speed runs with $C_T(J)$ enabled for S4 and S5, once E2
     lands.
   - Each scenario reports a scorecard: tracking RMS, percentage of time
     saturated, peak tilt, energy used.
4. **Model validity gates.** A SITL result is reported for a stratum only
   when the model includes every term that stratum marks ● in
   [section E](#e-aerodynamic-model-terms). Otherwise the scorecard is
   flagged *model-limited*.
5. **Hardware validation.**
   - Bench: thrust stand, then HIL on the flight controller.
   - Flight: S3 is the program's flight target (RP-1). The other strata are
     validated in simulation unless a vehicle becomes available.

## Recommended order of work

1. **A1 mixer**, then **B1/C1/D2 cascaded PID**, then **B2 filters**. This is
   the racing-drone Phase 1 path to gate P1, which flies S3 hardware slowly.
2. **A2 and A3** (thrust linearization and motor lag), **D1** (the CTBR
   interface), and **F1/F2**. These are needed before S3 flies fast.
3. **E1 rotor drag and E2 thrust lapse** in the model, then the S3 SITL
   gates at speed.
4. **B3 INDI** and **B4 gain scheduling** as the high-speed rate loop.
5. **E3, E6, G2, G3**, only if S4 or S5 become program targets.

## References

1. Bitcraze, *Crazyflie 2.1 datasheet* — <https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf>
2. Bitcraze, *Crazyflie 2.1 Brushless* — <https://bitcraze.io/products/crazyflie-2-1-brushless>
3. GetFPV, *BetaFPV Meteor65 1S Brushless Whoop* — <https://www.getfpv.com/betafpv-meteor65-1s-brushless-whoop-quadcopter-22000kv.html>
4. DJI, *Mini 4 Pro specifications* — <https://www.dji.com/mini-4-pro/specs>
5. TU Delft MAVLab, *MonoRace* (arXiv 2601.15222): 966 g, 5.1" props, τ = 25 ms, 28.23 m/s, ≈ 7 g, 16 g IMU limit — <https://arxiv.org/abs/2601.15222>
6. Drone Champions League, *The drones* (850–1000 g, ≥ 360 mm diagonal, > 160 km/h) — <https://dcl.aero/explore/drones/>
7. UAVModel, *Best FPV drone motors — thrust testing*; MAD BSC 2207.5 listing (1.85 kg) — <https://blog.uavmodel.com/6111-2/>
8. DroneDJ, *Red Bull drone chases F1* (350 km/h, < 1 kg, 42 000 rpm, ≈ 3 min) — <https://dronedj.com/2024/02/29/red-bull-drone-f1-fastest/>; Newsshooter — <https://www.newsshooter.com/2024/02/27/worlds-fastest-camera-drone-300-km-h-in-just-4-seconds-with-a-top-speed-of-over-350-km-h/>
9. LIGA.net, *Peregreen 3 reaches 585 km/h* (2.77 kg, 16.2 kW, water cooling) — <https://tech.liga.net/en/technology/news/drone-printed-on-a-3d-printer-reached-speed-of-585-kmh---video>
10. New Atlas, *Peregreen V4 reclaims speed record* (T-Motor 3120 900 KV, 6" props) — <https://newatlas.com/drones/luke-mike-bell-peregreen-v4-guinness-speed/>
11. Inspenet, *Peregreen 2 Guinness record* (Velox 3115 900 KV, 12S, 120 A ESCs) — <https://inspenet.com/en/news/quadcopter-record-guinness-speed-recorder/>; Guinness World Records, 480.23 km/h — <https://www.guinnessworldrecords.com/news/2024/5/father-and-son-duo-create-worlds-fastest-drone-that-reaches-500-km-h>
12. Tom's Hardware, *Peregreen V4 at 657.59 km/h* — <https://www.tomshardware.com/3d-printing/fully-3d-printed-drone-capable-of-flying-at-408-mph-is-the-fastest-in-the-world-father-son-duo-reclaim-guinness-world-record-title-with-diy-quadcopter>
13. *Data-Driven System Identification of Quadrotors Subject to Motor Delays* (Crazyflie τ ≈ 0.072 s; earlier estimate 0.065 s) — <https://arxiv.org/abs/2404.07837>
14. Bitcraze, *PWM to thrust* and *Keeping thrust consistent as the battery drains* — <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/>
15. *Abu Dhabi Autonomous Racing League* (A2RL drones > 140 km/h) — <https://en.wikipedia.org/wiki/Abu_Dhabi_Autonomous_Racing_League>
