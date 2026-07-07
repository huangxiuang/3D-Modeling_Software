import rasterio
import numpy as np
import pyvista as pv

file_path = 'ASTGTM_N46E129H.img'

with rasterio.open(file_path) as src:
    data = src.read(1).astype(np.float32)

data[data < -50] = 0.0 
data[data > 9000] = 0.0 

step = 2
data = data[::step, ::step]
rows, cols = data.shape
print(f"降采样后的网格大小: {rows}行 x {cols}列")

pixel_size = 30.0 * step 
x = np.arange(cols, dtype=np.float32) * pixel_size
y = np.arange(rows, dtype=np.float32) * pixel_size
xx, yy = np.meshgrid(x, y)

# z坐标被*10，以增加可视化效果
exaggeration = 10
exaggerated_z = data * exaggeration 

points = np.column_stack((xx.ravel(), yy.ravel(), exaggerated_z.ravel()))
grid = pv.StructuredGrid()
grid.points = points.astype(np.float32)
grid.dimensions = (cols, rows, 1)

plotter = pv.Plotter()

plotter.add_mesh(grid, 
                 cmap='gist_earth', 
                 scalars=data.ravel(), 
                 clim=(0, 800),      
                 smooth_shading=True, 
                 lighting=True,
                 show_edges=False)


plotter.show_bounds(grid='front', location='outer', all_edges=True, show_zlabels=False)

plotter.show_grid(color='lightgrey') 


plotter.show_axes() 


print("渲染中... 已恢复坐标轴刻度。")
plotter.show()