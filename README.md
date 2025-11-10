# Navigatio Mmesh System With Pathfinding

This is the first step of my custom **Navigation Mesh** system, which also includes a **Pathfinding algorithm**. For now, it doesn’t have **heightfields**, but my next goal is to implement it.
<img width="2548" height="1387" alt="bothpath" src="https://github.com/user-attachments/assets/8a39fe28-766e-4ef3-872b-8a5ad703af32" />
- For cloning repo `git clone https://github.com/haktan313/NavigationmeshSystem.git`
- Then run the Generation.bat file

🧩 Some Features
- ImGui panel for building the navmesh based on agent radius, debug options, and start/end position adjustment.
- Triangles are created by **ear clipping**. I used a custom raycast for ear clipping.
- **Hertel Mehlhorn** algorithm for optimizing triangles and creating complex poligons, which reduces the number of nodes needed for pathfinding.
- A* **pathfinding** combined with the **Simple Stupid Funnel Algorithm** for path smoothing.

## 📸 Screenshots
| NavMesh (Before Optimization) | NavMesh (After Hertel–Mehlhorn Optimization) |
|--------|-------------|
| <img width="2554" height="1384" alt="triangleswithsmoothpath" src="https://github.com/user-attachments/assets/945f73cf-c581-4436-bd6f-a0ec5a21bd29" /> | <img width="2549" height="1385" alt="justnavmesh" src="https://github.com/user-attachments/assets/3364a358-0c86-42a8-891f-77c5656612d0" /> |

| Normal Path | Smoothed Path (Funnel Algorithm) |
|--------|-------------|
| <img width="2549" height="1391" alt="Screenshot 2025-11-10 113245" src="https://github.com/user-attachments/assets/7408df89-b97d-414f-a9bf-2bf8a7f2919c" /> | <img width="2542" height="1377" alt="Screenshot 2025-11-10 113400" src="https://github.com/user-attachments/assets/c4b90997-5b73-450d-b58f-4651101f0ac1" /> |
