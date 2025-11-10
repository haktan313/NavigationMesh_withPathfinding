# Navigatio Mmesh System With Pathfinding

This is the first step of my custom **Navigation Mesh** system, which also includes a **Pathfinding algorithm**. For now, it doesn’t have **heightfields**, but my next goal is to implement it.
<img width="2548" height="1387" alt="bothpath" src="https://github.com/user-attachments/assets/873549cc-d569-426b-932b-6c14a28bb2dd" />
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
| <img width="2554" height="1384" alt="triangleswithsmoothpath" src="https://github.com/user-attachments/assets/e59d8700-b5f7-47a3-8ff4-22fdf85169db" /> | <img width="2549" height="1385" alt="justnavmesh" src="https://github.com/user-attachments/assets/f356ffe0-7a74-4c8e-aa17-328be0f619aa" /> |

| Normal Path | Smoothed Path (Funnel Algorithm) |
|--------|-------------|
| <img width="2549" height="1391" alt="Screenshot 2025-11-10 113245" src="https://github.com/user-attachments/assets/5fdaf817-ddbc-41c0-9af3-7eaf37117d06" /> | <img width="2542" height="1377" alt="image" src="https://github.com/user-attachments/assets/10265a99-94a0-4b15-bc9e-76d16c6596d7" /> |
