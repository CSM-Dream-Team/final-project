use flight::mesh::{MeshSource, VertN, Indexing, Primitive};

/// Creates a cube with seemingly rounded edges. Because real edges are always
/// somewhat rounded, this cube will have more realistic reflections. The
/// resulting mesh is composed of 24 verticies and 44 triangles.
/// 
/// * `rad` - radius of the cube, half the length of a side
/// * `bev` - radius of the bevel (should be less than `rad`).
pub fn beveled_cube(rad: f32, bev: f32) -> MeshSource<VertN, ()> {
    let verts = vec![
        VertN { pos: [rad, -(rad - bev), -(rad - bev)], norm: [1., 0., 0.] },
        VertN { pos: [(rad - bev), -rad, -(rad - bev)], norm: [0., -1., 0.] },
        VertN { pos: [(rad - bev), -(rad - bev), -rad], norm: [0., 0., -1.] },
        VertN { pos: [rad, -(rad - bev), (rad - bev)], norm: [1., 0., 0.] },
        VertN { pos: [(rad - bev), -(rad - bev), rad], norm: [0., 0., 1.] },
        VertN { pos: [(rad - bev), -rad, (rad - bev)], norm: [0., -1., 0.] },
        VertN { pos: [-(rad - bev), -(rad - bev), rad], norm: [0., 0., 1.] },
        VertN { pos: [-rad, -(rad - bev), (rad - bev)], norm: [-1., 0., 0.] },
        VertN { pos: [-(rad - bev), -rad, (rad - bev)], norm: [0., -1., 0.] },
        VertN { pos: [-(rad - bev), -(rad - bev), -rad], norm: [0., 0., -1.] },
        VertN { pos: [-(rad - bev), -rad, -(rad - bev)], norm: [0., -1., 0.] },
        VertN { pos: [-rad, -(rad - bev), -(rad - bev)], norm: [-1., 0., 0.] },
        VertN { pos: [(rad - bev), (rad - bev), -rad], norm: [0., 0., -1.] },
        VertN { pos: [(rad - bev), rad, -(rad - bev)], norm: [0., 1., 0.] },
        VertN { pos: [rad, (rad - bev), -(rad - bev)], norm: [1., 0., 0.] },
        VertN { pos: [(rad - bev), (rad - bev), rad], norm: [0., 0., 1.] },
        VertN { pos: [rad, (rad - bev), (rad - bev)], norm: [1., 0., 0.] },
        VertN { pos: [(rad - bev), rad, (rad - bev)], norm: [0., 1., 0.] },
        VertN { pos: [-rad, (rad - bev), (rad - bev)], norm: [-1., 0., 0.] },
        VertN { pos: [-(rad - bev), (rad - bev), rad], norm: [0., 0., 1.] },
        VertN { pos: [-(rad - bev), rad, (rad - bev)], norm: [0., 1., 0.] },
        VertN { pos: [-rad, (rad - bev), -(rad - bev)], norm: [-1., 0., 0.] },
        VertN { pos: [-(rad - bev), rad, -(rad - bev)], norm: [0., 1., 0.] },
        VertN { pos: [-(rad - bev), (rad - bev), -rad], norm: [0., 0., -1.] },
    ]; 
    
    let inds = vec![3-1, 24-1, 13-1, 6-1, 11-1, 2-1, 19-1, 12-1, 8-1, 23-1,
        18-1, 14-1, 16-1, 7-1, 5-1, 1-1, 2-1, 3-1, 4-1, 5-1, 6-1, 7-1, 8-1, 9-1,
        10-1, 11-1, 12-1, 13-1, 14-1, 15-1, 16-1, 17-1, 18-1, 19-1, 20-1, 21-1,
        22-1, 23-1, 24-1, 4-1, 2-1, 1-1, 11-1, 3-1, 2-1, 13-1, 1-1, 3-1, 7-1, 6-1,
        5-1, 17-1, 5-1, 4-1, 12-1, 9-1, 8-1, 20-1, 8-1, 7-1, 22-1, 10-1, 12-1, 18-1,
        15-1, 14-1, 24-1, 14-1, 13-1, 21-1, 16-1, 18-1, 23-1, 19-1, 21-1, 15-1, 4-1,
        1-1, 3-1, 10-1, 24-1, 6-1, 9-1, 11-1, 19-1, 22-1, 12-1, 23-1, 21-1, 18-1,
        16-1, 20-1, 7-1, 4-1, 6-1, 2-1, 11-1, 10-1, 3-1, 13-1, 15-1, 1-1, 7-1, 9-1,
        6-1, 17-1, 16-1, 5-1, 12-1, 11-1, 9-1, 20-1, 19-1, 8-1, 22-1, 24-1, 10-1,
        18-1, 17-1, 15-1, 24-1, 23-1, 14-1, 21-1, 20-1, 16-1, 23-1, 22-1, 19-1,
        15-1, 17-1, 4-1]; 
    
    MeshSource {
        verts: verts,
        inds: Indexing::Inds(inds),
        prim: Primitive::TriangleList,
        mat: (),
    }
}
