#include <cmath>
#include <cstdint>
#include <algorithm>
#include <array>

// ============================================================================
//   body frame: x = forward(+), y = right(+), z = down(+)
//   roll  input: degrees, POSITIVE = right side down
//   pitch input: degrees, POSITIVE = nose down
// ============================================================================

namespace ge {

constexpr int    kN            = 8;       // Array size, 8x8 zones
constexpr double kHFOV_deg     = 45.0;    // [deg] FoV
constexpr double kVFOV_deg     = 45.0;    // [deg] FoV
constexpr double kPsiS_deg     = 135.0;   // [deg] sensor mounting yaw

constexpr double kXs           = 0.018;   // [m] sensor offset from CoM, +forward
constexpr double kYs           = 0.008;   // [m] sensor offset from CoM, +right
constexpr double kZs           = 0.0;     // [m] sensor offset from CoM, +down

constexpr double kRProp        = 0.045;   // [m] propeller radius
constexpr double kRhoGE        = 3.5;     // [-] empirical GE coefficient
constexpr double kMass         = 0.877;   // [kg]
constexpr double kG            = 9.81;    // [m/s^2] gravity
constexpr double kThrottleHover= 0.55;    // [0~1] hover throttle

constexpr double kHMin         = 0.05;    // [m] clearance floor (avoid singularity)
constexpr double kHThreshold   = 4.0 * kRProp;  // [m] beyond this, GE ~negligible
constexpr double kEth          = 0.03;    // [m] RMS plane-fit validity threshold -- tune
constexpr int    kNMin         = 16;      // min valid zones for a trustworthy fit
constexpr double kDropFrac     = 0.2;     // RANSAC-lite: fraction of worst zones dropped
constexpr double kRMinValid    = 0.02;    // [m] sensor min usable range
constexpr double kRMaxValid    = 4.0;     // [m] sensor max usable range

// Rotor arm offsets, X-config, x=forward(+) y=right(+).
constexpr double kArmLen       = 0.050;    // [m]
constexpr double kArmProj      = kArmLen * 0.70710678118654752;  // L/sqrt(2)

struct RotorPos { double x, y; const char* label; };
constexpr RotorPos kRotors[4] = {
    { +kArmProj, +kArmProj, "FR (front-right)" },
    { +kArmProj, -kArmProj, "FL (front-left)"  },
    { -kArmProj, +kArmProj, "RR (rear-right)"  },
    { -kArmProj, -kArmProj, "RL (rear-left)"   },
};

inline double deg2rad(double d) { return d * M_PI / 180.0; }

struct Vec3 { double x, y, z; };

inline Vec3 rotZ(const Vec3& v, double psi) {
    double c = std::cos(psi), s = std::sin(psi);
    return { c*v.x - s*v.y, s*v.x + c*v.y, v.z };
}

struct ZoneDirTable {
    Vec3 d[64];
    ZoneDirTable() {
        const double hfov = deg2rad(kHFOV_deg);
        const double vfov = deg2rad(kVFOV_deg);
        const double psi_s = deg2rad(kPsiS_deg);
        for (int iy = 0; iy < kN; ++iy) {
            double ty = -vfov/2.0 + (iy + 0.5) * vfov / kN;
            for (int ix = 0; ix < kN; ++ix) {
                double tx = -hfov/2.0 + (ix + 0.5) * hfov / kN;
                double denom = std::sqrt(1.0 + std::tan(tx)*std::tan(tx)
                                              + std::tan(ty)*std::tan(ty));
                Vec3 dsensor { std::tan(tx)/denom, std::tan(ty)/denom, 1.0/denom };
                d[iy*kN + ix] = rotZ(dsensor, psi_s);   // fold in mounting yaw once
            }
        }
    }
};
static const ZoneDirTable kZoneDirs;

inline void Rx(double phi, double R[3][3]) {
    double c = std::cos(phi), s = std::sin(phi);
    R[0][0]=1; R[0][1]=0;  R[0][2]=0;
    R[1][0]=0; R[1][1]=c;  R[1][2]=-s;
    R[2][0]=0; R[2][1]=s;  R[2][2]=c;
}
inline void RyCorrected(double theta, double R[3][3]) {
    double c = std::cos(theta), s = std::sin(theta);
    R[0][0]=c;  R[0][1]=0; R[0][2]=-s;
    R[1][0]=0;  R[1][1]=1; R[1][2]=0;
    R[2][0]=s;  R[2][1]=0; R[2][2]=c;
}
inline void matmul3(const double A[3][3], const double B[3][3], double Cm[3][3]) {
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j) {
            double s=0;
            for (int k=0;k<3;++k) s += A[i][k]*B[k][j];
            Cm[i][j]=s;
        }
}
inline Vec3 matvec3(const double R[3][3], const Vec3& v) {
    return { R[0][0]*v.x+R[0][1]*v.y+R[0][2]*v.z,
             R[1][0]*v.x+R[1][1]*v.y+R[1][2]*v.z,
             R[2][0]*v.x+R[2][1]*v.y+R[2][2]*v.z };
}

struct Plane { double a=0, b=0, c=0; bool ok=false; };

inline Plane fitPlane(const Vec3* pts, const int* idx, int n) {
    Plane out;
    if (n < 3) return out;

    double Sxx=0, Sxy=0, Sx=0, Syy=0, Sy=0, S1=0;
    double Sxz=0, Syz=0, Sz=0;
    for (int k=0; k<n; ++k) {
        const Vec3& p = pts[idx[k]];
        Sxx += p.x*p.x; Sxy += p.x*p.y; Sx += p.x;
        Syy += p.y*p.y; Sy += p.y;      S1 += 1.0;
        Sxz += p.x*p.z; Syz += p.y*p.z; Sz += p.z;
    }
    // Solve [Sxx Sxy Sx; Sxy Syy Sy; Sx Sy S1] * [a b c]^T = [Sxz Syz Sz]^T
    double M[3][3] = {{Sxx,Sxy,Sx},{Sxy,Syy,Sy},{Sx,Sy,S1}};
    double rhs[3]  = {Sxz, Syz, Sz};

    // 3x3 closed-form inverse via cofactors
    double det = M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
               - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
               + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
    if (std::fabs(det) < 1e-12) return out;

    double inv[3][3];
    inv[0][0] =  (M[1][1]*M[2][2]-M[1][2]*M[2][1]) / det;
    inv[0][1] = -(M[0][1]*M[2][2]-M[0][2]*M[2][1]) / det;
    inv[0][2] =  (M[0][1]*M[1][2]-M[0][2]*M[1][1]) / det;
    inv[1][0] = -(M[1][0]*M[2][2]-M[1][2]*M[2][0]) / det;
    inv[1][1] =  (M[0][0]*M[2][2]-M[0][2]*M[2][0]) / det;
    inv[1][2] = -(M[0][0]*M[1][2]-M[0][2]*M[1][0]) / det;
    inv[2][0] =  (M[1][0]*M[2][1]-M[1][1]*M[2][0]) / det;
    inv[2][1] = -(M[0][0]*M[2][1]-M[0][1]*M[2][0]) / det;
    inv[2][2] =  (M[0][0]*M[1][1]-M[0][1]*M[1][0]) / det;

    out.a = inv[0][0]*rhs[0] + inv[0][1]*rhs[1] + inv[0][2]*rhs[2];
    out.b = inv[1][0]*rhs[0] + inv[1][1]*rhs[1] + inv[1][2]*rhs[2];
    out.c = inv[2][0]*rhs[0] + inv[2][1]*rhs[1] + inv[2][2]*rhs[2];
    out.ok = true;
    return out;
}

struct GEResult {
    bool   valid = false;
    double alpha_rad = 0.0;
    double h_eff = 0.0;
    double e_rms = 0.0;
    int    n_valid = 0;
    double dT_ff = 0.0;
    double h_motor[4]   = {0,0,0,0}; // per-rotor clearance, order = kRotors
    double dT_motor[4]  = {0,0,0,0}; // per-rotor feedforward correction
};

inline GEResult computeGroundEffect(const float tof_m[64], float roll_deg, float pitch_deg) {
    GEResult res;

    const double roll  = deg2rad(roll_deg);
    const double pitch = deg2rad(pitch_deg);

    double Rx_[3][3], Ry_[3][3], R[3][3];
    Rx(roll, Rx_);
    RyCorrected(pitch, Ry_);
    matmul3(Ry_, Rx_, R);     // R = Ry(pitch) * Rx(roll)

    Vec3 p_local[64];
    bool valid_range[64];
    int n_range_valid = 0;

    const Vec3 t_s { kXs, kYs, kZs };
    for (int i = 0; i < 64; ++i) {
        double r = tof_m[i];
        valid_range[i] = std::isfinite(r) && (r > kRMinValid) && (r < kRMaxValid);
        Vec3 p_sensor_origin { r * kZoneDirs.d[i].x, r * kZoneDirs.d[i].y, r * kZoneDirs.d[i].z };
        Vec3 p_body { p_sensor_origin.x + t_s.x, p_sensor_origin.y + t_s.y, p_sensor_origin.z + t_s.z };
        p_local[i] = matvec3(R, p_body);
        if (valid_range[i]) ++n_range_valid;
    }

    if (n_range_valid < kNMin) {
        res.valid = false;
        res.n_valid = n_range_valid;
        return res;
    }

    int idxAll[64]; int nAll = 0;
    for (int i = 0; i < 64; ++i) if (valid_range[i]) idxAll[nAll++] = i;

    Plane pl = fitPlane(p_local, idxAll, nAll);
    if (!pl.ok) { res.valid = false; res.n_valid = nAll; return res; }

    double resid[64];
    for (int k = 0; k < nAll; ++k) {
        const Vec3& p = p_local[idxAll[k]];
        resid[k] = p.z - (pl.a*p.x + pl.b*p.y + pl.c);
    }
    int order[64];
    for (int k = 0; k < nAll; ++k) order[k] = k;
    std::sort(order, order + nAll, [&](int i1, int i2) {
        return std::fabs(resid[i1]) < std::fabs(resid[i2]);
    });
    int keepN = std::max(kNMin, (int)(nAll * (1.0 - kDropFrac)));
    int idxKeep[64];
    for (int k = 0; k < keepN; ++k) idxKeep[k] = idxAll[order[k]];

    pl = fitPlane(p_local, idxKeep, keepN);
    if (!pl.ok) { res.valid = false; res.n_valid = keepN; return res; }

    double sse = 0;
    for (int k = 0; k < keepN; ++k) {
        const Vec3& p = p_local[idxKeep[k]];
        double e = p.z - (pl.a*p.x + pl.b*p.y + pl.c);
        sse += e*e;
    }
    double e_rms = std::sqrt(sse / keepN);

    bool valid = (e_rms < kEth) && (keepN >= kNMin);

    double denom = std::sqrt(pl.a*pl.a + pl.b*pl.b + 1.0);
    double alpha = std::acos(1.0 / denom);
    double h_eff = std::fabs(pl.c) / denom;

    res.valid   = valid;
    res.alpha_rad = alpha;
    res.h_eff   = h_eff;
    res.e_rms   = e_rms;
    res.n_valid = keepN;

    const double k1 = kRhoGE * (kMass * kG) / (kMass * kG / kThrottleHover);
    if (valid && h_eff < kHThreshold) {
        double h_c = std::max(h_eff, kHMin);
        res.dT_ff = -k1 * std::pow(kRProp / (4.0 * h_c), 2.0) * std::cos(alpha);
    } else {
        res.dT_ff = 0.0;
    }

    if (valid) {
        for (int m = 0; m < 4; ++m) {
            double hk = std::fabs(pl.a*kRotors[m].x + pl.b*kRotors[m].y + pl.c) / denom;
            res.h_motor[m] = hk;
            if (hk >= kHThreshold) {
                res.dT_motor[m] = 0.0;
            } else {
                double hc = std::max(hk, kHMin);
                res.dT_motor[m] = -k1 * std::pow(kRProp / (4.0 * hc), 2.0) * std::cos(alpha);
            }
        }
    }

    return res;
}
}