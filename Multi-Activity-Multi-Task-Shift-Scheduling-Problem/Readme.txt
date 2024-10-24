# Parse Instruction

This document provides instructions on how to parse the data for the Multi-Activity Multi-Task Shift Scheduling Problem.

<sequence_length>
<primary_flag>  #Always 1

EMPL
<number_of_employees>
<employee_details>:
    number_of_employees: An integer representing the number of employees.
    employee_details: For each employee, the following details are provided:
        Number of pieces
        Piece start times and lengths
        Number of skills and the skills themselves

ACT
<number_of_activities>
<activity_details>:
    number_of_activities: An integer representing the number of activities.
    activity_details: For each activity, the following details are provided:
        Minimum and maximum activity lengths
        Demand for each sequence position
        Over and under coverage costs for each sequence position

TASK
<number_of_tasks>
<task_details>:
    number_of_tasks: An integer representing the number of tasks.
    task_details: For each task, the following details are provided:
        Task length, over and under coverage costs
        Start and length of the task's time window
        Number of precedence constraints and the tasks they depend on