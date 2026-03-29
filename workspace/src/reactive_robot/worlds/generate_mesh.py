import math

def generate_5_obj(filename="shape_5.obj"):
    vertices = []
    normals = []
    faces = []

    def normalize(v):
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag == 0: return (0,0,1)
        return (v[0]/mag, v[1]/mag, v[2]/mag)

    def add_box(cx, cy, cz, l, w, h, yaw):
        nonlocal vertices, normals, faces
        start_idx = len(vertices) + 1
        hl, hw, hh = l/2, w/2, h/2
        
        corners = [
            (hl, hw, -hh), (-hl, hw, -hh), (-hl, -hw, -hh), (hl, -hw, -hh),
            (hl, hw, hh), (-hl, hw, hh), (-hl, -hw, hh), (hl, -hw, hh)
        ]
        
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        
        for x, y, z in corners:
            # Rotate and translate vertices
            rx = x * cos_y - y * sin_y
            ry = x * sin_y + y * cos_y
            vx, vy, vz = rx + cx, ry + cy, z + cz
            vertices.append((vx, vy, vz))
            
            # Calculate an outward-facing normal for physics collisions
            nx, ny, nz = normalize((rx, ry, z))
            normals.append((nx, ny, nz))
            
        faces.append((start_idx+3, start_idx+2, start_idx+1, start_idx+0)) 
        faces.append((start_idx+4, start_idx+5, start_idx+6, start_idx+7)) 
        faces.append((start_idx+2, start_idx+3, start_idx+7, start_idx+6)) 
        faces.append((start_idx+0, start_idx+1, start_idx+5, start_idx+4)) 
        faces.append((start_idx+3, start_idx+0, start_idx+4, start_idx+7)) 
        faces.append((start_idx+1, start_idx+2, start_idx+6, start_idx+5)) 

    # --- Dimensions ---
    thickness = 0.35
    height = 1.0
    
    add_box(-0.0, 5.0, 0.5, 3.0, thickness, height, 0)
    
    add_box(-1.5, 3.5, 0.5, thickness, 3.35, height, 0)
    
    add_box(-0.65, 2.0, 0.5, 1.7, thickness, height, 0)

    num_segments = 100
    start_angle = math.radians(90)
    end_angle = math.radians(-135)
    radius = 2.0
    
    angle_step = (end_angle - start_angle) / (num_segments - 1)
    seg_length = radius * abs(angle_step) * 1.5
    
    for i in range(num_segments):
        angle = start_angle + i * angle_step
        cx = radius * math.cos(angle)
        cy = radius * math.sin(angle)
        yaw = angle + math.pi/2
        add_box(cx, cy, 0.5, seg_length, thickness, height, yaw)

    with open(filename, "w") as f:
        f.write("# Auto-generated Smooth Shape 5 OBJ with Physics Normals\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for vn in normals:
            f.write(f"vn {vn[0]:.6f} {vn[1]:.6f} {vn[2]:.6f}\n")
        for face in faces:
            f.write(f"f {face[0]}//{face[0]} {face[1]}//{face[1]} {face[2]}//{face[2]} {face[3]}//{face[3]}\n")

    print(f"Successfully generated {filename} with physics normals!")

if __name__ == '__main__':
    generate_5_obj()