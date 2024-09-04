## Description of the Code Files

This repository contains the implementation of the Mixed Integer Programming **(MILP)**, **(Constraint Programming)** formulations, **Constructive_Heuristic**, Descent Heuristic with Shaking **(DSH)** and **Repair_Method** algorithms presented in the paper [The Dial-a-Tour Problem](https://www.sciencedirect.com/science/article/abs/pii/S0305054824003046).

### Files and Structure

- ```main.cpp```: Main entry point for the application. It coordinates the flow of the program and integrates all components.
- ```MILPSolver.cpp```: Contains *MILP formulation* using the ILOG CPLEX Solver v20.1
- ```CPSolver.cpp```: Includes the two Constraint Programming formulations (*CP1* and *CP2*) using the ILOG CP optimizer
- ```Greedy.cpp```: Provides the *Constructive Heuristic*, based on a Greedy procedure
- ```Local Search.cpp```: Includes the *DHS* and *Repair_Method* algorithms

- ```Instance.cpp```: This class holds the characteristics of the benchmark instances 
- ```Solver.cpp```: Parent class for all the present Solvers (Solution Methods) with all its common arguments.
- ```Solution.cpp```: Created to transform the output of a solver into the solution class structure.
- ```parameter.cpp```: Contains all the necessary parameters present in our benchmark instances.


  *All the previous files appear with their respective header (.hpp) files, where the necessary classes and methods are declared for their use.*
