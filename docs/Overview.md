```mermaid
graph TD;
    Problem-->Setup;
    Planner-->Setup;
    Optimizer-->Planner;
    CollisionChecker-->Planner;
    Space;
    Boundaries;
```