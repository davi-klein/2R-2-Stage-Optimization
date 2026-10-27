{\rtf1\ansi\ansicpg1252\cocoartf2870
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 # 2R Serial Robot: Trajectory Optimization and PID Control\
\
This repository contains the official MATLAB and Simulink codebase for the manuscript detailing two-stage trajectory optimization and PID controller gain tuning for 2R serial robots. The study rigorously evaluates three metaheuristic algorithms: Firefly Metaheuristic Algorithm (FMA), Dhole Optimization Algorithm (DOA), and Particle Swarm Optimization (PSO).\
\
## Repository Architecture\
\
*   **`/src`**: Contains the metaheuristic optimization algorithms (`fma.m`, `doa.m`, `pso.m`), the objective functions for position error and Dynamic Residual Torque (DRT) minimization, and all analytical plotting scripts.\
*   **`/models`**: Houses the high-fidelity dynamic plant model `Robot_2R_Dynamics_and_Control.slx`.\
*   **`/data`**: Stores the high-resolution reference trajectories (`trajetoria2R_high_res.mat`).\
*   **`/results`**: The designated output directory for the 30-run statistical benchmark data (`/statistical_benchmark`) and the publication-ready IEEE-styled plots (`/figures`).\
\
## Execution Pipeline (`main.m`)\
\
The primary orchestration script (`main.m`) provides an interactive command-line interface to execute single runs, launch the parallelized statistical benchmark, or generate the manuscript's figures.\
\
**Single-Run Optimizations (Options 1\'966)**\
Executes a single tuning trial for a chosen algorithm (FMA, DOA, or PSO) and cost function formulation (Position Error or DRT). These options compile the Simulink cache, initialize a parallel pool, and save the resulting optimal gains to the `/results` directory.\
\
**Statistical Benchmark Suite (Option 7)**\
Launches a fully automated, 30-run statistical benchmark across all six combinations of algorithms and cost functions. This option implements strict memory sanitation, assigns deterministic RNG seeds (1000, 2000, and 3000 series), and leverages isolated parallel worker caches to prevent read-write collisions during the 180 total simulations.\
\
**Publication Analytics & Plotting (Option 8)**\
Routes to a dedicated sub-menu for generating the methodological and statistical visualizations required for the manuscript.\
*   **1. Stage 1 Optimal Trajectory Profile:** Generates the kinematic profiles (position, velocity, acceleration, and torque) of the optimized reference trajectory alongside actuator saturation limits.\
*   **2. Ablation Study:** Compares the constrained optimization framework against an unconstrained quintic B-spline baseline to demonstrate convergence trapping and torque limit violations.\
*   **3. Statistical Optimizer Selection:** Outputs algorithm dispersion boxplots, computational effort bar charts, descriptive statistics matrices, and Bonferroni-Holm corrected Wilcoxon rank-sum tests to identify the most robust optimizer.\
*   **4. Physical Metrics & Trade-off Evaluations:** Simulates the champion controllers to extract physical performance metrics (e.g., RMSE, control effort) and generates the zoomed bivariate Pareto frontier mapping tracking precision against actuator effort.\
\
## Software Dependencies\
\
*   MATLAB (R2023a or newer recommended)\
*   Simulink\
*   Parallel Computing Toolbox\
*   Global Optimization Toolbox (required for the PSO baseline comparisons)}