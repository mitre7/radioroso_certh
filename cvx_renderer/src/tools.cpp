#include "tools.hpp"

//#include <certh_core/GeomUtil.h>
#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;
using namespace cvx::renderer ;

static float polygon_area(const vector<Vector2f> &contour)
{
    uint n = contour.size();

    float A = 0.0f;

    for( uint p=n-1, q=0; q<n; p=q++ )
    {
        A+= contour[p].x()*contour[q].y() - contour[q].x()*contour[p].y();
    }

    return A*0.5f;
}

/*
     InsideTriangle decides if a point P is Inside of the triangle
     defined by A, B, C.
   */
bool inside_triangle(float Ax, float Ay,
                                 float Bx, float By,
                                 float Cx, float Cy,
                                 float Px, float Py)

{
    float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
    float cCROSSap, bCROSScp, aCROSSbp;

    ax = Cx - Bx;  ay = Cy - By;
    bx = Ax - Cx;  by = Ay - Cy;
    cx = Bx - Ax;  cy = By - Ay;
    apx= Px - Ax;  apy= Py - Ay;
    bpx= Px - Bx;  bpy= Py - By;
    cpx= Px - Cx;  cpy= Py - Cy;

    aCROSSbp = ax*bpy - ay*bpx;
    cCROSSap = cx*apy - cy*apx;
    bCROSScp = bx*cpy - by*cpx;

    return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}

bool snip(const vector<Vector2f> &contour, int u,int v,int w,int n, vector<uint> &V)
{
    static const float EPSILON = 1.0e-10f;

    uint p;
    float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

    Ax = contour[V[u]].x();
    Ay = contour[V[u]].y();

    Bx = contour[V[v]].x();
    By = contour[V[v]].y();

    Cx = contour[V[w]].x();
    Cy = contour[V[w]].y();

    if ( EPSILON > (((Bx-Ax)*(Cy-Ay)) - ((By-Ay)*(Cx-Ax))) ) return false;

    for ( p=0 ; p<n ; p++ )
    {
        if ( (p == u) || (p == v) || (p == w) ) continue;
        Px = contour[V[p]].x();
        Py = contour[V[p]].y();
        if ( inside_triangle(Ax,Ay,Bx,By,Cx,Cy,Px,Py) ) return false;
    }

    return true;
}

static float triangle_area(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3) {
    Vector3f n1, n2 ;

    n1 = v1 - v2 ;
    n2 = v1 - v3 ;
    return  n1.cross(n2).norm() ;
}

Eigen::Hyperplane<float, 3> fitPlaneToPoints(const vector<Vector3f> &pts) {
    uint n_pts = pts.size() ;
    assert(n_pts >= 3) ;

    if ( n_pts == 3 )
        return Eigen::Hyperplane<float, 3>::Through(pts[0], pts[1], pts[2]) ;
    else {
        Eigen::Map<Matrix<float,Dynamic,3,RowMajor> > mat((float *)pts.data(), pts.size(), 3);
        VectorXf centroid = mat.colwise().mean();
        MatrixXf centered = mat.rowwise() - centroid.adjoint() ;
        MatrixXf cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
        JacobiSVD<Matrix3f> svd(cov, ComputeFullU);
        Vector3f normal = svd.matrixU().col(2);
        return Eigen::Hyperplane<float, 3>(normal, centroid) ;
    }
}


bool triangulate(const vector<Vector3f> &pts, vector<uint32_t> &result)
{
    if ( pts.size() < 3 ) return false ;

    if ( pts.size() == 3 ) {

        result.push_back(0) ; result.push_back(1) ; result.push_back(2) ;
        return true ;
    }

    // fit plane to points

    Eigen::Hyperplane<float, 3> plane = fitPlaneToPoints(pts) ;

    Vector3f na, nb, nz = plane.normal() ;
    double q = sqrt(nz.x() * nz.x() + nz.y() * nz.y()) ;
    if ( q < 1.0e-4 )
    {
        na = Vector3f(1, 0, 0) ;
        nb = nz.cross(na) ;
    }
    else {
        na = Vector3f(nz.y()/q, -nz.x()/q, 0) ;
        nb = Vector3f(nz.x() * nz.z()/q, nz.y() * nz.z()/q, -q) ;
    }

    // project points to plane

    vector<Vector2f> contour ;
    for(uint i=0 ; i<pts.size() ; i++ ) {
        const Vector3f &pt = pts[i] ;
        contour.push_back(Vector2f(na.dot(pt), nb.dot(pt))) ;
    }

    /* allocate and initialize list of Vertices in polygon */

    uint n = contour.size();

    vector<uint> V(n) ;

    /* we want a counter-clockwise polygon in V */

    if (  polygon_area(contour) > 0 )
        for ( uint v=0; v<n; v++) V[v] = v;
    else
        for(int v=0; v<n; v++) V[v] = (n-1)-v;

    uint nv = n;

    /*  remove nv-2 Vertices, creating 1 triangle every time */
    uint count = 2*nv;   /* error detection */

    for(int m=0, v=nv-1; nv>2; )
    {
        /* if we loop, it is probably a non-simple polygon */
        if (0 >= (count--))
        {
            //** Triangulate: ERROR - probable bad polygon!
            return false;
        }

        /* three consecutive vertices in current polygon, <u,v,w> */
        uint u = v  ; if (nv <= u) u = 0;     /* previous */
        v = u+1; if (nv <= v) v = 0;     /* new v    */
        uint w = v+1; if (nv <= w) w = 0;     /* next     */

        if ( snip(contour,u,v,w,nv,V) )
        {
            uint a,b,c,s,t;

            /* true names of the vertices */
            a = V[u]; b = V[v]; c = V[w];

            /* output Triangle */
            result.push_back( a );
            result.push_back( b );
            result.push_back( c );

            m++;

            /* remove v from remaining polygon */
            for(s=v,t=v+1;t<nv;s++,t++) V[s] = V[t]; nv--;

            /* resest error detection counter */
            count = 2*nv;
        }
    }


    return true;
}


void flatten_mesh(const Mesh &mesh, std::vector<Vector3f> &vertices, std::vector<Vector3f> &normals, std::vector<Vector3f> &colors,
                  std::vector<Vector2f> tex_coords[])
{
    vector<Vector3f> cnormals ;

    if ( mesh.normals_.empty() )
        compute_normals(mesh.vertices_, mesh.vertex_indices_, cnormals );

    for( uint v=0 ; v<mesh.vertex_indices_.size() ; v++) {

        uint32_t vidx = mesh.vertex_indices_[v] ;
        const Vector3f &pos = mesh.vertices_[vidx] ;
        vertices.push_back(pos) ;

        if ( !mesh.normal_indices_.empty() ) {
            uint32_t nidx = mesh.normal_indices_[v] ;
            const Vector3f &normal = mesh.normals_[nidx] ;
            normals.push_back(normal) ;
        }

        if ( !cnormals.empty() ) {
            const Vector3f &normal = cnormals[vidx] ;
            normals.push_back(normal) ;
        }
        else {
            const Vector3f &norm = mesh.normals_[vidx] ;
            normals.push_back(norm) ;
        }

        if ( !mesh.colors_.empty() ) {
            if ( !mesh.color_indices_.empty() ) {
                uint32_t cidx = mesh.color_indices_[v] ;
                const Vector3f &color = mesh.colors_[cidx] ;
                colors.push_back(color) ;
            }
            else {
                const Vector3f &color = mesh.colors_[vidx] ;
                colors.push_back(color) ;
            }
        }

        for( uint t=0 ; t<MAX_TEXTURES ; t++ ) {
            if ( !mesh.tex_coords_[t].empty() ) {
                if ( !mesh.tex_coord_indices_[t].empty() ) {
                    uint32_t tidx = mesh.tex_coord_indices_[t][v] ;
                    const Vector2f &uv = mesh.tex_coords_[t][tidx] ;
                    tex_coords[t].push_back(uv) ;
                }
                else {
                    const Vector2f &uv = mesh.tex_coords_[t][vidx] ;
                    tex_coords[t].push_back(uv) ;
                }
            }
        }
    }
}


static Vector3f normal_triangle(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3)
{
    Vector3f n1, n2 ;

    n1 = v1 - v2 ;
    n2 = v1 - v3 ;
    return  n1.cross(n2).normalized() ;

}
void compute_normals(const vector<Vector3f> &vertices, const vector<uint> &indices, vector<Vector3f> &vtx_normals)
{
    vtx_normals.resize(vertices.size()) ;
    for( int i=0 ; i<vertices.size() ; i++ ) vtx_normals[i] = Vector3f::Zero() ;

    for( int i=0 ; i<indices.size() ; i+=3 )
    {
        uint idx0 = indices[i] ;
        uint idx1 = indices[i+1] ;
        uint idx2 = indices[i+2] ;
        Vector3f n = normal_triangle(vertices[idx0], vertices[idx1], vertices[idx2]) ;

        vtx_normals[idx0] += n ;
        vtx_normals[idx1] += n ;
        vtx_normals[idx2] += n ;
    }

    for( int i=0 ; i<vertices.size() ; i++ ) vtx_normals[i].normalize() ;

}
