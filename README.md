# Multiple Item Capacitated Lot Sizing Problem (CLSP)

A **Mixed Integer Linear Programming (MILP)** model in **Python** for the **Multi-Item Capacitated Lot Sizing Problem**, built with the **[Pyomo](http://www.pyomo.org/)** optimization framework and solved via the **IBM ILOG CPLEX** solver.

## Overview

The Capacitated Lot Sizing Problem (CLSP) is a classic production planning problem in Operations Research. Given a set of items, a planning horizon divided into discrete periods, and a shared production resource with limited capacity, the goal is to decide **how much of each item to produce in each period** in order to satisfy known demand while minimizing total cost — without exceeding capacity in any period.

This repository contains a working Python implementation of the multi-item variant, where several products compete for the same capacitated resource in every time period.

## Repository Contents

| File | Description |
|---|---|
| `Multy_Item_Capacitated_Lot_Sizing_Problem.py` | Python script implementing and solving the CLSP via Pyomo and CPLEX |
| `Multi_item_Capacitated_Lot-Sizing_(CLSP)_Problem_Results.txt` | Solver output: status, optimal objective value, and decision variable values |
| `Multy Item Capacitated Lot Sizing Problem (CLSP).pdf` | Mathematical formulation of the problem |

## Mathematical Formulation

### Sets
- $T$ = the planning horizon (index $t = 0, 1, \dots, n$)
- $J$ = set of items (index $j = 1, \dots, m$)

### Parameters
- $d_{tj}$ = the demand forecast at time $t$ for item $j$
- $c_{tj}$ = the unit production or purchasing cost at time $t$ for item $j$
- $h_{tj}$ = the unit inventory cost at time $t$ for item $j$
- $K_j$ = the fixed setup or ordering cost for item $j$
- $C_{tj}$ = the maximum feasible lot size (capacity) at time $t$ for item $j$

### Variables
- $q_{tj}$ = quantity to be produced or ordered during period $t$ for item $j$
- $I_{tj}$ = inventory level at the end of period $t$ for item $j$
- $y_{tj}$ = binary setup variable:

$$
y_{tj} = \begin{cases} 1 & \text{if units of item } j \text{ are manufactured/ordered in period } t \\ 0 & \text{otherwise} \end{cases}
$$

### Objective Function

**(1)**

$$
\displaystyle \min \sum_{t=1}^{n} \sum_{j=1}^{m} \left( K_j \cdot y_{tj} + c_{tj} \cdot q_{tj} + h_{tj} \cdot I_{tj} \right)
$$

### Constraints

**(2)** — Inventory is zero at the start and at the end of the horizon

$$
I_{tj} = 0 \qquad t = 0 \ \text{and} \ t = n,\ \ \forall j \in J
$$

**(3)** — Demand satisfaction and inventory balance

$$
q_{tj} + I_{t-1,j} = d_{tj} + I_{tj} \qquad \forall t \in T \setminus \{0\},\ \ \forall j \in J
$$

**(4)** — Production is capped by capacity and linked to the setup decision

$$
q_{tj} \le C_{tj} \cdot y_{tj} \qquad \forall t \in T \setminus \{0\},\ \ \forall j \in J
$$

**(5)** — Non-negative production

$$
q_{tj} \ge 0 \qquad \forall t \in T \setminus \{0\},\ \ \forall j \in J
$$

**(6)** — Non-negative inventory

$$
I_{tj} \ge 0 \qquad \forall t \in T \setminus \{0\},\ \ \forall j \in J
$$

**(7)** — Binary setup variable

$$
y_{tj} \in \{0,1\} \qquad \forall t \in T \setminus \{0\},\ \ \forall j \in J
$$

### Interpretation

The objective function (1) represents the total management costs, including production (and/or purchasing), inventory, and setup/ordering costs. Conditions (2) impose that inventory levels at the beginning and end of the planning horizon are equal to zero. Constraints (3) reproduce the demand satisfaction and inventory balance constraint for each period. Constraints (4)–(5) allow positive production (bounded between 0 and the period capacity $C_{tj}$) if and only if the setup variable $y_{tj}$ is equal to 1.

A copy of this formulation is also available as a standalone PDF in this repository.

## Example Instance

The script ships with a sample instance featuring:

- **3 items**, **5 demand periods** (plus an initial period $t = 0$)
- Per-period production capacity: **100 units** (uniform across items and periods)
- Setup costs: **400** (item 1), **150** (item 2), **100** (item 3)
- Holding costs: **4 / 3 / 2** per unit per period (items 1, 2, 3)
- Unit production costs: **0** (not considered in this instance)
- Demand is sparse: concentrated in periods 1, 3, and 5

## Requirements

Install the required Python packages via pip:

```bash
pip install pyomo numpy pandas
```

**IBM ILOG CPLEX** must also be installed separately on your system. An academic license is available free of charge through the [IBM Academic Initiative](https://www.ibm.com/academic).

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/Diego-Fabbri/Multiple_Item_Capacitated_Lot_Sizing_Problem_-CLSP_Py.git
   cd Multiple_Item_Capacitated_Lot_Sizing_Problem_-CLSP_Py
   ```

2. Run the script:
   ```bash
   python Multy_Item_Capacitated_Lot_Sizing_Problem.py
   ```

## Output

When executed, the script:
- Builds the MILP model using Pyomo's `ConcreteModel`
- Solves it via CPLEX and prints the full model structure to the console
- Writes the results to `Multi_item_Capacitated_Lot-Sizing_(CLSP)_Problem_Results.txt`, including:
  - Solver status and termination condition
  - Total optimal cost (objective value)
  - Production quantity $q_{tj}$, inventory level $I_{tj}$, and setup decision $y_{tj}$ for each item and period

## License

No license has been specified for this repository. Please contact the author before reusing this code for purposes beyond personal study or reference.
