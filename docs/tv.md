# Torque vectoring algorithms for performance-oriented 4WD electric race cars

**Four independently driven electric motors create a uniquely over-actuated vehicle that can generate yaw moments, maximize tire utilization, and reshape handling characteristics entirely through software** — delivering lap time improvements of **5–9%** over non-vectored configurations. The dominant control architecture across both academic research and Formula Student competition is a three-layer hierarchy: a reference generator computing target yaw rate from driver inputs, a high-level controller producing a corrective yaw moment demand, and a low-level allocator distributing torque to each wheel via constrained optimization. This report synthesizes the algorithms, vehicle models, performance strategies, and real-world implementations that define the state of the art.

---

## The three-layer control architecture that dominates the field

Nearly all torque vectoring (TV) implementations — from university Formula Student cars to Rimac's production hypercars — follow a hierarchical structure with three distinct functional layers.

**Layer 1, the reference generator**, computes a target yaw rate from driver steering input and vehicle speed. The steady-state yaw rate reference derived from the linear bicycle model is:

$$\dot{\psi}_{ref} = \frac{V \cdot \delta}{L \cdot (1 + K_{US} \cdot V^2)}$$

where $V$ is vehicle speed, $\delta$ is front steering angle, $L$ is wheelbase, and $K_{US}$ is the understeer gradient. For performance applications, De Novellis, Sorniotti, and Gruber (IEEE TVT, 2014) extended this into a nonlinear piecewise formulation that allows three tunable objectives: reducing the understeer gradient for quicker response, extending the linear handling region to higher lateral accelerations, and increasing the maximum achievable lateral acceleration $a_{y,max}$. The maximum yaw rate is physically bounded by friction: $\dot{\psi}_{max} = \mu g / V$. A key finding from Smith et al. (Cranfield, AVEC 2016) showed that the **time-optimal yaw rate reference coincides exactly with the natural steady-state single-track model expression** — deviating from this by ±0.5 deg/g in understeer gradient cost approximately 0.65 s per lap per degree-per-g of deviation.

**Layer 2, the high-level controller**, computes the total drive torque demand from the driver's throttle input and a corrective yaw moment $M_z$ from the error between reference and measured yaw rate. This is where the choice of control algorithm — PID, sliding mode, MPC, LQR, H∞, or fuzzy logic — matters most.

**Layer 3, the control allocator**, solves the over-actuated distribution problem: mapping two virtual demands ($F_{x,total}$, $M_z$) to four independent wheel torques. The simplified yaw moment equation from differential torque is:

$$M_z = \frac{t}{2 R_w} (\Delta T_{front} + \Delta T_{rear})$$

where $t$ is track width, $R_w$ is wheel radius, and $\Delta T$ is the torque difference between right and left wheels on each axle. Because four actuators serve only two or three control objectives (longitudinal force, yaw moment, and optionally lateral force management), the system has infinite solutions for any given demand — making optimization essential.

---

## Control algorithms: from PID baselines to predictive optimization

**Direct Yaw Moment Control (DYC) with PID feedback** represents the baseline. A proportional-integral-derivative controller acts on yaw rate error to produce $M_z$, which is then distributed as differential left-right torque. Combined with a feedforward component derived from the bicycle model, this architecture delivers **5–9% lap time improvement** on skidpad tests (De Pascale, Lenzo et al., AIMETA 2019). The feedforward term handles approximately 80–90% of the required yaw moment during normal cornering, providing instant response to steering input with no measurement delay, while the feedback corrects transient errors and disturbances. De Novellis et al. (2014) showed that the feedforward component is essential for shaping steady-state handling, while the feedback type primarily affects transient robustness.

**Sliding Mode Control (SMC)** defines a sliding surface on the yaw rate tracking error, typically $\sigma = e_r + \lambda \int e_r \, dt$, and applies a discontinuous switching control law. The second-order super-twisting algorithm eliminates chattering while maintaining robustness:

$$u = -k_1 |\sigma|^{1/2} \text{sign}(\sigma) + v, \quad \dot{v} = -k_2 \text{sign}(\sigma)$$

Goggia, Sorniotti, and Ferrara (IEEE TVT, 2015) experimentally validated integral sliding mode TV on a prototype EV, demonstrating effective performance despite real-world CAN bus delays and actuator dynamics. SMC's main advantage for racing is **robustness against model uncertainties** — tire wear, temperature shifts, and changing track conditions — with very low computational cost. Its disadvantage is potential chattering that can excite drivetrain vibrations, though boundary-layer and higher-order variants largely solve this.

**Model Predictive Control (MPC)** solves an optimal control problem over a finite prediction horizon at each timestep:

$$J = \sum_{k=0}^{N_p-1} \left[ (x_k - x_{ref})^T Q (x_k - x_{ref}) + u_k^T R \, u_k \right]$$

subject to motor torque limits, friction circle constraints ($\sqrt{F_{x,i}^2 + F_{y,i}^2} \leq \mu F_{z,i}$), total power budgets, and torque rate limits. Šulc et al. (IEEE, 2018) compared linear time-varying MPC and nonlinear MPC for a Formula Student car, finding that LTV-MPC with QP solvers running at 25–50 ms sample rates is feasible in real time on embedded hardware. For energy-limited Formula Student racing (**80 kW power cap, <7 kWh battery**), Maull and Schommer (World EVJ, 2022) showed MPC-based torque optimization yielded **1.99 seconds (13.5%) lap time improvement** over a constant-torque approach at equivalent energy consumption. MPC's unique advantage is explicit constraint handling — it can simultaneously respect motor limits, friction limits, and power budgets while optimizing performance.

**Fuzzy logic controllers** encode expert knowledge as linguistic rules mapping yaw rate error and its derivative to yaw moment demands. A direct comparison at Oxford Brookes University (Julve, Springer, 2025) found that fuzzy logic outperformed both PID and SMC in response speed, accuracy, and reduced oscillations for Formula Student TV. **Deep reinforcement learning**, particularly Twin-Delayed DDPG, formulates torque distribution as a Markov Decision Process with reward functions combining yaw rate tracking, sideslip angle penalties, and motor efficiency. These approaches are model-free and can handle highly nonlinear tire dynamics at the limit, but lack formal stability guarantees and remain unproven in competitive racing.

**The optimal torque allocation layer** is most commonly formulated as a quadratic program:

$$\min \, (Bu - v)^T W_1 (Bu - v) + u^T W_2 \, u$$

where $u = [T_1, T_2, T_3, T_4]^T$ is the torque vector, $v = [F_{x,total}, M_z]^T$ is the virtual demand, and $B$ is the effectiveness matrix. De Novellis et al. (SAE 2013-01-0673) found that **cost functions minimizing tire slip outperform energy-efficiency cost functions for performance applications** — a critical finding for racing. Minimizing the sum of squared normalized tire utilizations $\sum (F_{x,i} / \mu F_{z,i})^2$ equalizes friction usage across all four wheels, leaving maximum margin for lateral forces.

---

## Vehicle dynamics models and tire force representation

The **linear bicycle model** (2-DOF) serves primarily as the reference generator. Its state-space form with sideslip angle $\beta$ and yaw rate $r$ as states incorporates tire cornering stiffnesses $C_f$, $C_r$ and is well-suited for reference computation below approximately **4 m/s² lateral acceleration**. Above this threshold, tire force saturation invalidates the linear assumption. The TV control input enters as an additional yaw moment term $[0; \, 1/I_{zz}] \cdot M_z$. This model cannot capture load transfer, combined slip, or asymmetric left-right conditions — precisely the phenomena that matter most at the racing limit.

The **full four-wheel (two-track) model** resolves forces at each tire independently. The yaw dynamics equation includes individual longitudinal and lateral forces at all four corners:

$$I_{zz} \dot{r} = [(F_{yfl} + F_{yfr})\cos\delta + (F_{xfl} + F_{xfr})\sin\delta] l_f - (F_{yrl} + F_{yrr}) l_r + \frac{b_f}{2}(F_{xfr} - F_{xfl}) + \frac{b_r}{2}(F_{xrr} - F_{xrl})$$

This model incorporates **lateral load transfer** through roll stiffness distribution. The vertical load at each wheel is computed as:

$$F_{z,ij} = F_{z,static} \pm \frac{m \cdot a_x \cdot h}{2L} \pm \frac{m \cdot a_y \cdot h \cdot K_{\phi,f/r}}{b \cdot K_{\phi,total}}$$

where $K_{\phi,f}$ and $K_{\phi,r}$ are front and rear roll stiffnesses. The **Total Lateral Load Transfer Distribution** (TLLTD = $\Delta F_{z,front} / \Delta F_{z,total}$) directly determines passive understeer/oversteer balance and is a primary mechanical tuning parameter that TV can effectively override in software.

For tire force representation, the **Pacejka Magic Formula** $y = D \sin\{C \arctan[Bx - E(Bx - \arctan(Bx))]\}$ remains the industry standard, with combined-slip weighting functions reducing pure-slip forces based on the presence of the other slip component. The **Dugoff model** is preferred for MPC and optimization-based controllers because of its analytical differentiability: $\lambda = \mu F_z (1-\sigma) / [2\sqrt{(C_\sigma \sigma)^2 + (C_\alpha \tan\alpha)^2}]$, with $f(\lambda) = (2-\lambda)\lambda$ for $\lambda < 1$. It requires fewer parameters and directly incorporates friction coefficient $\mu$, making it suitable for real-time applications.

**Tire force estimation** in racing EVs exploits a unique advantage: motor torque is precisely known, enabling direct longitudinal force computation from wheel dynamics ($F_x = (T_m - I_\omega \dot{\omega})/R$). Unscented Kalman Filters with random-walk tire force models (no explicit tire model needed) have demonstrated excellent tracking in combined-slip maneuvers. The Cubature Kalman Filter addresses numerical stability issues of UKF in high-dimensional state spaces and has been validated for distributed-drive EVs.

---

## Exploiting the friction circle for maximum cornering performance

The core physics enabling torque vectoring performance gains is the **coupled tire force constraint**: at each tire, $\sqrt{F_x^2 + F_y^2} \leq \mu F_z$. Applying longitudinal force consumes lateral force capacity according to $F_{y,available} = \sqrt{(\mu F_z)^2 - F_x^2}$, a relationship that is severely nonlinear — small longitudinal forces have minimal lateral penalty, but large ones dramatically reduce cornering grip.

During cornering, lateral load transfer creates **asymmetric friction circles**: outer tires have larger capacity (higher $F_z$), inner tires smaller. With equal torque distribution, inner tires saturate first while outer tires retain ~70% of their capacity. Park and Gerdes (Stanford, IEEE IV 2015) demonstrated that optimal tire force allocation equalizes the **friction usage ratio** $k_i = \sqrt{F_{x,i}^2 + F_{y,i}^2} / (\mu F_{z,i})$ across all four tires. In their analysis, a conventional front-steered vehicle showed inner tires at $k \approx 0.96$–0.99 and outer tires at $k \approx 0.70$–0.72; with optimal 4WD allocation, all tires converged to $k \approx 0.80$, preventing premature saturation and maximizing the total vehicle force envelope.

The quantitative performance improvements are substantial and well-documented across multiple independent studies:

- **+10.7% maximum lateral acceleration** (8.06 → 8.92 m/s²) with Sport-mode TV (Lenzo et al., Meccanica, 2021)
- **~9% skidpad lap time reduction** for a Formula Student car (De Pascale, Lenzo et al., 2019)
- **~2.1 seconds per race lap** on a full circuit for a RWD electric race car (Ghosh & Tonoli, SAE 2017-01-0509)
- **7.5% overall lap time improvement** for 4WD with TV vs. 2WD (MDPI WEVJ, 2023)

The mechanism is threefold: TV reduces or eliminates longitudinal force on lightly loaded inner tires (freeing their full friction capacity for lateral force), applies greater torque to heavily loaded outer tires (which can sustain combined forces efficiently), and generates a net yaw moment that supplements natural cornering forces — enabling front and rear tires to reach saturation simultaneously, which is the theoretical maximum cornering condition.

For **faster turn-in**, feed-forward yaw moment applied immediately upon steering input rotates the car into the corner before yaw rate error even develops. Electric motor response times of **<5–10 ms** (compared to 50–200 ms for hydraulic LSD actuation) mean the TV system acts faster than any mechanical differential. Schaeffler reported that their DTM Electric Democar's TV response was so rapid they had to deliberately slow it down to feel natural to drivers.

**Handling balance tuning** through torque vectoring effectively creates a **software-configurable understeer gradient**. Rear torque bias reduces front tire workload and decreases understeer; front bias increases it. Rimac's production system allows drivers to select understeer, neutral, or oversteer handling at the push of a button. This represents a paradigm shift: what traditionally required physical changes to springs, anti-roll bars, or ballast can now be adjusted between corners on a single lap.

---

## Race car integration: aero, thermal management, and energy strategy

**Aerodynamic downforce** fundamentally changes the TV optimization. Downforce scales with $V^2$, so at high speed the available grip may be **2–3× the static mechanical grip**. TV controllers must incorporate speed-dependent torque maps that account for the normal force at each wheel: $F_z = F_{z,static} + \frac{1}{2} C_L A \rho V^2 \cdot d_i$, where $d_i$ is the aero load distribution factor for each corner. If the aerodynamic balance differs from the static weight distribution — which is typical in race cars — the car's handling character shifts with speed, and TV can compensate dynamically.

**Tire thermal management** through torque distribution is increasingly important for endurance events. Race tires operate in optimal temperature windows as narrow as **5°C**, and TV inherently creates asymmetric thermal loading during cornering. The Thermo Racing Tire (TRT) model by Farroni et al. (2014, 2019) applies Fourier heat transfer through tread, carcass, and sidewall layers, and demonstrated that **grip correlates best with tread core temperature** rather than surface temperature. TV can manage tire degradation over a stint by equalizing work across tires to prevent asymmetric wear, reducing slip on overheated tires, and strategically biasing torque during straight-line running to allow overworked tires to cool.

For **energy-limited racing** (Formula Student's 80 kW cap, Formula E's battery constraints), torque distribution directly affects consumption. For four identical PMSMs, Prost et al. (AVEC 2024) proved that maximum drivetrain efficiency occurs when torque is equally shared — the efficiency-vs-split relationship is mathematically convex. Their experimental powertrain loss models from an electric hypercar showed **up to 32% efficiency improvement** over fixed 50:50 AWD through optimal allocation. Multi-objective optimization balancing yaw rate tracking, motor efficiency, tire slip losses, and torque smoothness achieved approximately **22% energy reduction** with dynamic programming versus fuzzy logic coordination.

**Trail braking with differential regenerative braking** is uniquely powerful in 4-motor EVs. Applying more regenerative braking to the inner wheel during turn-in creates a yaw-aiding moment while simultaneously **recovering energy** — thermodynamically superior to brake-based TV, which wastes energy as heat. The entire trail-braking sequence — differential regen for turn-in, modulated regen through mid-corner, transition to drive torque at exit — can be executed continuously through software with no mechanical lag.

---

## How TV compares to mechanical limited-slip differentials

Electronic torque vectoring with independent motors offers categorical advantages over every form of mechanical LSD:

- **Bidirectional torque transfer**: Mechanical LSDs can only restrict speed differences (clutch-type) or react proportionally to input torque (Torsen). TV can actively drive the outer wheel faster while braking the inner wheel — a capability impossible with any passive mechanical system.
- **Response time**: Electric motors achieve full torque reversal in <5 ms versus 50–200 ms for hydraulic clutch actuation. Torsen differentials are purely reactive with zero electronic lag but cannot proactively distribute torque.
- **Independent multi-axis control**: A mechanical system requires separate differentials for left-right and front-rear distribution. Four motors provide three simultaneous degrees of freedom.
- **Software configurability**: Where LSD preload and ramp angles require physical changes, TV characteristics can be modified between corners on a single lap.
- **Energy recovery**: Unlike any mechanical differential, TV recovers energy during braking phases.

The fundamental limitation of a locked differential — that it resists the natural speed difference between inner and outer wheels during cornering, adding **parasitic understeer** — simply does not exist with independent motors. TV can send more torque to the faster (outer) wheel, something no mechanical differential can accomplish.

---

## Real-world implementations proving the theory

**In Formula Student Electric**, torque vectoring has been the dominant technical differentiator since its introduction in 2011. AMZ Racing (ETH Zurich) set the 0–100 km/h world record of **1.513 seconds** (2016) with four 37 kW hub motors and torque vectoring, later broken by GreenTeam Stuttgart at **1.461 seconds** (2022) with a 145 kg, 180 kW car. AMZ's algorithm uses a simplified vehicle model to compute optimal yaw rate per corner, comparing this reference to IMU measurements and correcting via adjusted torque distribution at every wheel. FST Lisboa published experimentally validated results comparing PI and LQR controllers on their prototype (Robotics and Autonomous Systems, 2019). UninaCorse demonstrated the three-layer architecture with Pacejka tire models achieving ~9% skidpad improvement.

**Rimac's All-Wheel Torque Vectoring (R-AWTV)** system on the Nevera adjusts torque at each of four motors **100 times per second**, using accelerometers, gyroscopes, and forward-looking cameras with LIDAR to pre-compute optimal distribution. The system replaces traditional ABS and ESP entirely. The Tajima Rimac e-Runner completed Pikes Peak ahead of all ICE race cars, validating TV performance at the highest competitive level.

**The E-VECTOORC project** (EU FP7, 2011–2014), led by Prof. Aldo Sorniotti at the University of Surrey with 11 partners including Jaguar Land Rover, produced the most extensive experimental validation of TV algorithms on a four-wheel-drive Range Rover Evoque at the Lommel proving ground. Results included **28.8% brake distance reduction** and demonstrated that selectable understeer characteristics through TV are experimentally viable. This group — Surrey's Centre for Automotive Engineering — has published the largest body of work in the field, with control architectures spanning H∞ loop shaping, integral sliding mode, NMPC, and rule-based energy-efficient TV.

**Formula E Gen3 Evo** (2024–25 season) introduced the first internationally raced AWD electric formula car, deploying **50 kW from the front motor** during qualifying and Attack Mode alongside 350 kW at the rear. While this provides front-rear torque distribution rather than full left-right vectoring, it represents the highest-profile deployment of electric torque split in single-seater racing. Teams independently develop their proprietary control algorithms, and specific details remain closely guarded.

---

## Conclusion: the winning formula and remaining frontiers

The empirical evidence converges on a clear optimal approach for race car torque vectoring: **feedforward yaw moment from a bicycle-model reference generator combined with feedback correction (PID or SMC), feeding into a QP-based control allocator that minimizes tire friction utilization**. This architecture balances computational simplicity (critical for real-time execution at 1–10 ms cycle times) with strong performance. MPC represents the theoretical frontier — its explicit constraint handling and predictive capability offer additional performance, particularly for energy-limited racing, but computational demands currently limit it to 25–50 ms sample rates.

Three open challenges remain. First, **real-time tire state estimation at the friction limit** — current observers struggle precisely where TV matters most, at maximum lateral acceleration where tire models become least accurate. Second, **integrated trajectory optimization with TV** — coupling the path-planning problem with torque distribution, rather than treating them sequentially, could unlock additional lap time. Third, **thermal-aware allocation over full stint durations** — dynamically trading instantaneous performance against tire and motor thermal degradation represents a frontier that combines TV with race strategy optimization. The physics is clear: a 4-motor EV with well-implemented torque vectoring exploits tire capacity more completely than any mechanical system, and the gap between theoretical potential and practical implementation continues to narrow with every racing season.
