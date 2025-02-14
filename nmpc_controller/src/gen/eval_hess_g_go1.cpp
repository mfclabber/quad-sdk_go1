/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
#define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
#define _CASADI_NAMESPACE_CONCAT(NS, ID) NS##ID
#define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
#define CASADI_PREFIX(ID) eval_hess_g_go1_##ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#if defined(STATIC_LINKED)
#define CASADI_SYMBOL_EXPORT
#else
#define CASADI_SYMBOL_EXPORT __declspec(dllexport)
#endif
#elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
#define CASADI_SYMBOL_EXPORT __attribute__((visibility("default")))
#else
#define CASADI_SYMBOL_EXPORT
#endif
#endif

casadi_real casadi_sq(casadi_real x) { return x * x; }

static const casadi_int casadi_s0[40] = {
    36, 1,  0,  36, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
static const casadi_int casadi_s1[32] = {
    28, 1,  0,  28, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1,  2,  3,  4,
                                         5,  6, 7, 8,  9, 10, 11, 12, 13};
static const casadi_int casadi_s3[86] = {
    36, 36, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  4,  6,  8,  10, 12,
    14, 16, 18, 20, 22, 24, 26, 28, 30, 30, 30, 30, 35, 40, 41, 41, 41, 41,
    44, 46, 47, 27, 28, 27, 28, 27, 28, 28, 29, 28, 29, 28, 29, 28, 29, 28,
    29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 27, 28, 33,
    34, 35, 28, 29, 33, 34, 35, 29, 33, 34, 35, 34, 35, 35};

/* eval_hess_g_go1:(w[36],lambda[28],p[14])->(hess_g[36x36,47nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw,
                     casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108,
      a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119,
      a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13,
      a130, a131, a132, a133, a134, a135, a136, a14, a15, a16, a17, a18, a19,
      a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32,
      a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46,
      a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6,
      a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73,
      a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87,
      a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0 = arg[1] ? arg[1][11] : 0;
  a1 = 4.6361409999999998e-04;
  a2 = arg[0] ? arg[0][28] : 0;
  a3 = cos(a2);
  a4 = arg[0] ? arg[0][27] : 0;
  a5 = cos(a4);
  a6 = (a3 * a5);
  a6 = (a1 * a6);
  a7 = 2.3679520000000000e-04;
  a8 = sin(a4);
  a9 = (a3 * a8);
  a9 = (a7 * a9);
  a6 = (a6 - a9);
  a6 = (a0 * a6);
  a9 = arg[1] ? arg[1][10] : 0;
  a10 = (a1 * a8);
  a11 = (a7 * a5);
  a10 = (a10 + a11);
  a10 = (a9 * a10);
  a6 = (a6 - a10);
  a10 = (-a6);
  if (res[0] != 0) res[0][0] = a10;
  a10 = -3.1714528100000000e-02;
  a11 = cos(a2);
  a12 = (a10 * a11);
  a13 = cos(a4);
  a14 = sin(a2);
  a15 = (a13 * a14);
  a15 = (a7 * a15);
  a12 = (a12 - a15);
  a15 = sin(a4);
  a16 = (a15 * a14);
  a16 = (a1 * a16);
  a12 = (a12 - a16);
  a12 = (a0 * a12);
  a16 = (-a12);
  if (res[0] != 0) res[0][1] = a16;
  a16 = 3.0648510907999987e-01;
  a17 = (a3 * a5);
  a17 = (a16 * a17);
  a18 = 3.6671000000000002e-05;
  a19 = (a3 * a8);
  a19 = (a18 * a19);
  a17 = (a17 - a19);
  a17 = (a0 * a17);
  a19 = (a16 * a8);
  a20 = (a18 * a5);
  a19 = (a19 + a20);
  a19 = (a9 * a19);
  a17 = (a17 - a19);
  a17 = (-a17);
  if (res[0] != 0) res[0][2] = a17;
  a17 = -4.6361409999999998e-04;
  a19 = (a17 * a11);
  a20 = (a13 * a14);
  a20 = (a18 * a20);
  a19 = (a19 - a20);
  a20 = (a15 * a14);
  a20 = (a16 * a20);
  a19 = (a19 - a20);
  a19 = (a0 * a19);
  a19 = (-a19);
  if (res[0] != 0) res[0][3] = a19;
  a19 = (a3 * a5);
  a19 = (a18 * a19);
  a20 = 3.3002937628000001e-01;
  a21 = (a3 * a8);
  a21 = (a20 * a21);
  a19 = (a19 - a21);
  a19 = (a0 * a19);
  a21 = (a18 * a8);
  a22 = (a20 * a5);
  a21 = (a21 + a22);
  a21 = (a9 * a21);
  a19 = (a19 - a21);
  a19 = (-a19);
  if (res[0] != 0) res[0][4] = a19;
  a19 = -2.3679520000000000e-04;
  a21 = (a19 * a11);
  a22 = (a13 * a14);
  a22 = (a20 * a22);
  a21 = (a21 - a22);
  a22 = (a15 * a14);
  a22 = (a18 * a22);
  a21 = (a21 - a22);
  a21 = (a0 * a21);
  a21 = (-a21);
  if (res[0] != 0) res[0][5] = a21;
  a21 = arg[2] ? arg[2][0] : 0;
  a22 = arg[1] ? arg[1][9] : 0;
  a22 = (a21 * a22);
  a23 = arg[2] ? arg[2][3] : 0;
  a24 = (a23 * a11);
  a25 = arg[0] ? arg[0][29] : 0;
  a26 = sin(a25);
  a27 = arg[2] ? arg[2][4] : 0;
  a28 = (a27 * a14);
  a29 = (a26 * a28);
  a24 = (a24 - a29);
  a24 = (a22 * a24);
  if (res[0] != 0) res[0][6] = a24;
  a24 = (a27 * a3);
  a29 = cos(a25);
  a30 = (a24 * a29);
  a30 = (a22 * a30);
  a31 = (a21 * a9);
  a32 = sin(a25);
  a33 = (a27 * a32);
  a33 = (a31 * a33);
  a30 = (a30 - a33);
  if (res[0] != 0) res[0][7] = a30;
  a30 = arg[2] ? arg[2][2] : 0;
  a33 = (a30 * a11);
  a34 = cos(a25);
  a35 = (a27 * a14);
  a36 = (a34 * a35);
  a33 = (a33 - a36);
  a33 = (a22 * a33);
  a33 = (-a33);
  if (res[0] != 0) res[0][8] = a33;
  a33 = (a27 * a29);
  a33 = (a31 * a33);
  a36 = (a27 * a3);
  a37 = (a36 * a32);
  a37 = (a22 * a37);
  a33 = (a33 + a37);
  if (res[0] != 0) res[0][9] = a33;
  a33 = (a23 * a34);
  a37 = (a30 * a26);
  a33 = (a33 - a37);
  a37 = (a33 * a14);
  a37 = (a22 * a37);
  a37 = (-a37);
  if (res[0] != 0) res[0][10] = a37;
  a37 = (a23 * a32);
  a38 = (a30 * a29);
  a37 = (a37 + a38);
  a37 = (a3 * a37);
  a37 = (a22 * a37);
  a38 = (a23 * a29);
  a39 = (a30 * a32);
  a38 = (a38 - a39);
  a38 = (a31 * a38);
  a37 = (a37 + a38);
  a37 = (-a37);
  if (res[0] != 0) res[0][11] = a37;
  a37 = arg[2] ? arg[2][6] : 0;
  a38 = (a37 * a11);
  a39 = arg[2] ? arg[2][7] : 0;
  a40 = (a39 * a14);
  a41 = (a26 * a40);
  a38 = (a38 - a41);
  a38 = (a22 * a38);
  if (res[0] != 0) res[0][12] = a38;
  a38 = (a39 * a3);
  a41 = (a38 * a29);
  a41 = (a22 * a41);
  a42 = (a39 * a32);
  a42 = (a31 * a42);
  a41 = (a41 - a42);
  if (res[0] != 0) res[0][13] = a41;
  a41 = arg[2] ? arg[2][5] : 0;
  a42 = (a41 * a11);
  a43 = (a39 * a14);
  a44 = (a34 * a43);
  a42 = (a42 - a44);
  a42 = (a22 * a42);
  a42 = (-a42);
  if (res[0] != 0) res[0][14] = a42;
  a42 = (a39 * a29);
  a42 = (a31 * a42);
  a44 = (a39 * a3);
  a45 = (a44 * a32);
  a45 = (a22 * a45);
  a42 = (a42 + a45);
  if (res[0] != 0) res[0][15] = a42;
  a42 = (a37 * a34);
  a45 = (a41 * a26);
  a42 = (a42 - a45);
  a45 = (a42 * a14);
  a45 = (a22 * a45);
  a45 = (-a45);
  if (res[0] != 0) res[0][16] = a45;
  a45 = (a37 * a32);
  a46 = (a41 * a29);
  a45 = (a45 + a46);
  a45 = (a3 * a45);
  a45 = (a22 * a45);
  a46 = (a37 * a29);
  a47 = (a41 * a32);
  a46 = (a46 - a47);
  a46 = (a31 * a46);
  a45 = (a45 + a46);
  a45 = (-a45);
  if (res[0] != 0) res[0][17] = a45;
  a45 = arg[2] ? arg[2][9] : 0;
  a46 = (a45 * a11);
  a47 = arg[2] ? arg[2][10] : 0;
  a48 = (a47 * a14);
  a49 = (a26 * a48);
  a46 = (a46 - a49);
  a46 = (a22 * a46);
  if (res[0] != 0) res[0][18] = a46;
  a46 = (a47 * a3);
  a49 = (a46 * a29);
  a49 = (a22 * a49);
  a50 = (a47 * a32);
  a50 = (a31 * a50);
  a49 = (a49 - a50);
  if (res[0] != 0) res[0][19] = a49;
  a49 = arg[2] ? arg[2][8] : 0;
  a50 = (a49 * a11);
  a51 = (a47 * a14);
  a52 = (a34 * a51);
  a50 = (a50 - a52);
  a50 = (a22 * a50);
  a50 = (-a50);
  if (res[0] != 0) res[0][20] = a50;
  a50 = (a47 * a29);
  a50 = (a31 * a50);
  a52 = (a47 * a3);
  a53 = (a52 * a32);
  a53 = (a22 * a53);
  a50 = (a50 + a53);
  if (res[0] != 0) res[0][21] = a50;
  a50 = (a45 * a34);
  a53 = (a49 * a26);
  a50 = (a50 - a53);
  a53 = (a50 * a14);
  a53 = (a22 * a53);
  a53 = (-a53);
  if (res[0] != 0) res[0][22] = a53;
  a53 = (a45 * a32);
  a54 = (a49 * a29);
  a53 = (a53 + a54);
  a53 = (a3 * a53);
  a53 = (a22 * a53);
  a54 = (a45 * a29);
  a55 = (a49 * a32);
  a54 = (a54 - a55);
  a54 = (a31 * a54);
  a53 = (a53 + a54);
  a53 = (-a53);
  if (res[0] != 0) res[0][23] = a53;
  a53 = arg[2] ? arg[2][12] : 0;
  a54 = (a53 * a11);
  a55 = arg[2] ? arg[2][13] : 0;
  a56 = (a55 * a14);
  a57 = (a26 * a56);
  a54 = (a54 - a57);
  a54 = (a22 * a54);
  if (res[0] != 0) res[0][24] = a54;
  a54 = (a55 * a3);
  a57 = (a54 * a29);
  a57 = (a22 * a57);
  a58 = (a55 * a32);
  a58 = (a31 * a58);
  a57 = (a57 - a58);
  if (res[0] != 0) res[0][25] = a57;
  a57 = arg[2] ? arg[2][11] : 0;
  a58 = (a57 * a11);
  a59 = (a55 * a14);
  a60 = (a34 * a59);
  a58 = (a58 - a60);
  a58 = (a22 * a58);
  a58 = (-a58);
  if (res[0] != 0) res[0][26] = a58;
  a58 = (a55 * a29);
  a58 = (a31 * a58);
  a60 = (a55 * a3);
  a61 = (a60 * a32);
  a61 = (a22 * a61);
  a58 = (a58 + a61);
  if (res[0] != 0) res[0][27] = a58;
  a58 = (a53 * a34);
  a61 = (a57 * a26);
  a58 = (a58 - a61);
  a61 = (a58 * a14);
  a61 = (a22 * a61);
  a61 = (-a61);
  if (res[0] != 0) res[0][28] = a61;
  a61 = (a53 * a32);
  a62 = (a57 * a29);
  a61 = (a61 + a62);
  a61 = (a3 * a61);
  a61 = (a22 * a61);
  a29 = (a53 * a29);
  a32 = (a57 * a32);
  a29 = (a29 - a32);
  a29 = (a31 * a29);
  a61 = (a61 + a29);
  a61 = (-a61);
  if (res[0] != 0) res[0][29] = a61;
  a61 = arg[0] ? arg[0][34] : 0;
  a29 = arg[0] ? arg[0][35] : 0;
  a32 = (a61 * a29);
  a62 = (a32 * a3);
  a63 = (a21 * a0);
  a64 = (a1 * a63);
  a62 = (a62 * a64);
  a65 = arg[0] ? arg[0][33] : 0;
  a66 = (a65 * a29);
  a67 = (a66 * a3);
  a68 = 2.9831484817999998e-01;
  a69 = (a68 * a63);
  a67 = (a67 * a69);
  a62 = (a62 - a67);
  a67 = (a65 * a61);
  a70 = (a67 * a3);
  a71 = -3.6671000000000002e-05;
  a72 = (a71 * a63);
  a70 = (a70 * a72);
  a62 = (a62 + a70);
  a70 = casadi_sq(a29);
  a73 = (a7 * a63);
  a74 = (a70 * a73);
  a75 = (a3 * a74);
  a62 = (a62 + a75);
  a75 = casadi_sq(a65);
  a76 = (a7 * a63);
  a77 = (a75 * a76);
  a78 = (a3 * a77);
  a62 = (a62 - a78);
  a78 = arg[0] ? arg[0][11] : 0;
  a78 = (a29 - a78);
  a79 = (a78 * a0);
  a80 = (a18 * a79);
  a81 = (a3 * a80);
  a62 = (a62 + a81);
  a81 = arg[0] ? arg[0][10] : 0;
  a81 = (a61 - a81);
  a82 = (a81 * a0);
  a83 = (a16 * a82);
  a84 = (a3 * a83);
  a62 = (a62 + a84);
  a84 = arg[0] ? arg[0][9] : 0;
  a84 = (a65 - a84);
  a85 = (a84 * a0);
  a86 = (a1 * a85);
  a87 = (a3 * a86);
  a62 = (a62 + a87);
  a87 = (a61 * a29);
  a88 = (a7 * a31);
  a87 = (a87 * a88);
  a62 = (a62 + a87);
  a87 = (a65 * a29);
  a89 = (a71 * a31);
  a87 = (a87 * a89);
  a62 = (a62 + a87);
  a87 = (a65 * a61);
  a90 = 2.7477058097999990e-01;
  a91 = (a90 * a31);
  a87 = (a87 * a91);
  a62 = (a62 - a87);
  a87 = casadi_sq(a61);
  a92 = (a1 * a31);
  a93 = (a87 * a92);
  a62 = (a62 + a93);
  a93 = (a1 * a31);
  a94 = (a75 * a93);
  a62 = (a62 - a94);
  a78 = (a78 * a9);
  a94 = (a20 * a78);
  a62 = (a62 - a94);
  a81 = (a81 * a9);
  a94 = (a18 * a81);
  a62 = (a62 - a94);
  a84 = (a84 * a9);
  a94 = (a7 * a84);
  a62 = (a62 - a94);
  a94 = arg[1] ? arg[1][5] : 0;
  a94 = (a21 * a94);
  a95 = (a94 / a3);
  a96 = (a61 * a95);
  a62 = (a62 - a96);
  a96 = arg[1] ? arg[1][4] : 0;
  a96 = (a21 * a96);
  a97 = (a29 * a96);
  a62 = (a62 + a97);
  a97 = sin(a2);
  a98 = arg[1] ? arg[1][3] : 0;
  a21 = (a21 * a98);
  a98 = (a21 / a3);
  a99 = (a97 * a98);
  a100 = (a61 * a99);
  a62 = (a62 - a100);
  a100 = sin(a4);
  a62 = (a62 * a100);
  a100 = (a65 * a29);
  a101 = (a18 * a63);
  a102 = (a3 * a101);
  a103 = (a100 * a102);
  a104 = (a61 * a29);
  a105 = (a7 * a63);
  a106 = (a3 * a105);
  a107 = (a104 * a106);
  a103 = (a103 - a107);
  a107 = (a65 * a61);
  a90 = (a90 * a63);
  a108 = (a3 * a90);
  a109 = (a107 * a108);
  a103 = (a103 + a109);
  a109 = (a1 * a63);
  a110 = (a87 * a109);
  a111 = (a3 * a110);
  a103 = (a103 - a111);
  a111 = (a1 * a63);
  a112 = (a75 * a111);
  a113 = (a3 * a112);
  a103 = (a103 + a113);
  a113 = (a20 * a79);
  a114 = (a3 * a113);
  a103 = (a103 + a114);
  a114 = (a18 * a82);
  a115 = (a3 * a114);
  a103 = (a103 + a115);
  a115 = (a7 * a85);
  a116 = (a3 * a115);
  a103 = (a103 + a116);
  a116 = (a61 * a29);
  a117 = (a1 * a31);
  a116 = (a116 * a117);
  a103 = (a103 + a116);
  a116 = (a65 * a29);
  a68 = (a68 * a31);
  a116 = (a116 * a68);
  a103 = (a103 - a116);
  a116 = (a65 * a61);
  a118 = (a71 * a31);
  a116 = (a116 * a118);
  a103 = (a103 + a116);
  a116 = (a7 * a31);
  a119 = (a70 * a116);
  a103 = (a103 + a119);
  a119 = (a19 * a31);
  a75 = (a75 * a119);
  a103 = (a103 + a75);
  a78 = (a18 * a78);
  a103 = (a103 + a78);
  a81 = (a16 * a81);
  a103 = (a103 + a81);
  a84 = (a1 * a84);
  a103 = (a103 + a84);
  a84 = (a29 * a95);
  a103 = (a103 - a84);
  a84 = (a61 * a96);
  a103 = (a103 - a84);
  a84 = (a97 * a98);
  a81 = (a29 * a84);
  a103 = (a103 - a81);
  a81 = cos(a4);
  a103 = (a103 * a81);
  a62 = (a62 + a103);
  a62 = (-a62);
  if (res[0] != 0) res[0][30] = a62;
  a62 = cos(a4);
  a103 = (a66 * a14);
  a103 = (a69 * a103);
  a81 = (a32 * a14);
  a81 = (a64 * a81);
  a103 = (a103 - a81);
  a81 = (a67 * a14);
  a81 = (a72 * a81);
  a103 = (a103 - a81);
  a81 = (a74 * a14);
  a103 = (a103 - a81);
  a81 = (a77 * a14);
  a103 = (a103 + a81);
  a81 = (a80 * a14);
  a103 = (a103 - a81);
  a81 = (a83 * a14);
  a103 = (a103 - a81);
  a81 = (a86 * a14);
  a103 = (a103 - a81);
  a81 = (a95 / a3);
  a81 = (a81 * a14);
  a78 = (a61 * a81);
  a103 = (a103 - a78);
  a78 = (a98 / a3);
  a78 = (a78 * a14);
  a75 = (a97 * a78);
  a75 = (a21 + a75);
  a75 = (a61 * a75);
  a103 = (a103 - a75);
  a103 = (a62 * a103);
  a4 = sin(a4);
  a75 = (a105 * a14);
  a75 = (a104 * a75);
  a120 = (a101 * a14);
  a121 = (a100 * a120);
  a75 = (a75 - a121);
  a121 = (a90 * a14);
  a122 = (a107 * a121);
  a75 = (a75 - a122);
  a122 = (a110 * a14);
  a75 = (a75 + a122);
  a122 = (a112 * a14);
  a75 = (a75 - a122);
  a122 = (a113 * a14);
  a75 = (a75 - a122);
  a122 = (a114 * a14);
  a75 = (a75 - a122);
  a122 = (a115 * a14);
  a75 = (a75 - a122);
  a81 = (a29 * a81);
  a75 = (a75 - a81);
  a81 = (a97 * a78);
  a81 = (a21 + a81);
  a81 = (a29 * a81);
  a75 = (a75 - a81);
  a75 = (a4 * a75);
  a103 = (a103 - a75);
  if (res[0] != 0) res[0][31] = a103;
  a103 = (a72 * a5);
  a103 = (a3 * a103);
  a103 = (a61 * a103);
  a75 = (a69 * a5);
  a75 = (a3 * a75);
  a75 = (a29 * a75);
  a103 = (a103 - a75);
  a75 = (a102 * a8);
  a75 = (a29 * a75);
  a103 = (a103 - a75);
  a75 = (a108 * a8);
  a75 = (a61 * a75);
  a103 = (a103 - a75);
  a75 = (a89 * a5);
  a75 = (a29 * a75);
  a103 = (a103 + a75);
  a75 = (a91 * a5);
  a75 = (a61 * a75);
  a103 = (a103 - a75);
  a75 = (a68 * a8);
  a75 = (a29 * a75);
  a103 = (a103 + a75);
  a75 = (a118 * a8);
  a75 = (a61 * a75);
  a103 = (a103 - a75);
  a75 = (a65 + a65);
  a81 = (a3 * a8);
  a81 = (a111 * a81);
  a122 = (a3 * a5);
  a122 = (a76 * a122);
  a81 = (a81 + a122);
  a5 = (a93 * a5);
  a81 = (a81 + a5);
  a8 = (a119 * a8);
  a81 = (a81 + a8);
  a81 = (a75 * a81);
  a103 = (a103 - a81);
  a103 = (a103 + a6);
  if (res[0] != 0) res[0][32] = a103;
  a103 = (a3 * a29);
  a103 = (a64 * a103);
  a6 = (a3 * a65);
  a6 = (a72 * a6);
  a103 = (a103 + a6);
  a6 = (a16 * a0);
  a81 = (a3 * a6);
  a103 = (a103 + a81);
  a81 = (a88 * a29);
  a103 = (a103 + a81);
  a81 = (a91 * a65);
  a103 = (a103 - a81);
  a81 = (a61 + a61);
  a8 = (a92 * a81);
  a103 = (a103 + a8);
  a8 = (a18 * a9);
  a103 = (a103 - a8);
  a103 = (a103 - a95);
  a103 = (a103 - a99);
  a103 = (a62 * a103);
  a99 = (a108 * a65);
  a8 = (a106 * a29);
  a99 = (a99 - a8);
  a8 = (a109 * a81);
  a5 = (a3 * a8);
  a99 = (a99 - a5);
  a5 = (a18 * a0);
  a122 = (a3 * a5);
  a99 = (a99 + a122);
  a122 = (a117 * a29);
  a99 = (a99 + a122);
  a122 = (a118 * a65);
  a99 = (a99 + a122);
  a122 = (a16 * a9);
  a99 = (a99 + a122);
  a99 = (a99 - a96);
  a99 = (a4 * a99);
  a103 = (a103 - a99);
  if (res[0] != 0) res[0][33] = a103;
  a103 = (a3 * a61);
  a103 = (a64 * a103);
  a99 = (a3 * a65);
  a99 = (a69 * a99);
  a103 = (a103 - a99);
  a99 = (a29 + a29);
  a122 = (a73 * a99);
  a123 = (a3 * a122);
  a103 = (a103 + a123);
  a123 = (a18 * a0);
  a124 = (a3 * a123);
  a103 = (a103 + a124);
  a124 = (a88 * a61);
  a103 = (a103 + a124);
  a124 = (a89 * a65);
  a103 = (a103 + a124);
  a124 = (a20 * a9);
  a103 = (a103 - a124);
  a103 = (a103 + a96);
  a62 = (a62 * a103);
  a103 = (a102 * a65);
  a96 = (a106 * a61);
  a103 = (a103 - a96);
  a96 = (a20 * a0);
  a124 = (a3 * a96);
  a103 = (a103 + a124);
  a124 = (a117 * a61);
  a103 = (a103 + a124);
  a124 = (a68 * a65);
  a103 = (a103 - a124);
  a124 = (a116 * a99);
  a103 = (a103 + a124);
  a9 = (a18 * a9);
  a103 = (a103 + a9);
  a103 = (a103 - a95);
  a103 = (a103 - a84);
  a4 = (a4 * a103);
  a62 = (a62 - a4);
  if (res[0] != 0) res[0][34] = a62;
  a62 = (a61 * a29);
  a4 = -2.3544267200000080e-02;
  a4 = (a4 * a63);
  a62 = (a62 * a4);
  a103 = (a65 * a29);
  a84 = (a1 * a63);
  a103 = (a103 * a84);
  a62 = (a62 + a103);
  a103 = (a65 * a61);
  a95 = (a7 * a63);
  a103 = (a103 * a95);
  a62 = (a62 - a103);
  a103 = (a18 * a63);
  a70 = (a70 * a103);
  a62 = (a62 + a70);
  a71 = (a71 * a63);
  a87 = (a87 * a71);
  a62 = (a62 + a87);
  a79 = (a19 * a79);
  a62 = (a62 + a79);
  a82 = (a17 * a82);
  a62 = (a62 + a82);
  a10 = (a10 * a85);
  a62 = (a62 + a10);
  a10 = arg[0] ? arg[0][22] : 0;
  a85 = (a10 * a22);
  a82 = (a57 * a85);
  a62 = (a62 - a82);
  a82 = arg[0] ? arg[0][21] : 0;
  a79 = (a82 * a22);
  a87 = (a53 * a79);
  a62 = (a62 + a87);
  a87 = arg[0] ? arg[0][19] : 0;
  a63 = (a87 * a22);
  a70 = (a49 * a63);
  a62 = (a62 - a70);
  a70 = arg[0] ? arg[0][18] : 0;
  a9 = (a70 * a22);
  a124 = (a45 * a9);
  a62 = (a62 + a124);
  a124 = arg[0] ? arg[0][16] : 0;
  a125 = (a124 * a22);
  a126 = (a41 * a125);
  a62 = (a62 - a126);
  a126 = arg[0] ? arg[0][15] : 0;
  a127 = (a126 * a22);
  a128 = (a37 * a127);
  a62 = (a62 + a128);
  a128 = arg[0] ? arg[0][12] : 0;
  a129 = (a128 * a22);
  a130 = (a23 * a129);
  a62 = (a62 + a130);
  a130 = arg[0] ? arg[0][13] : 0;
  a131 = (a130 * a22);
  a132 = (a30 * a131);
  a62 = (a62 - a132);
  a132 = (a61 * a15);
  a133 = (a132 * a98);
  a62 = (a62 - a133);
  a133 = (a29 * a13);
  a134 = (a133 * a98);
  a62 = (a62 - a134);
  a134 = sin(a2);
  a62 = (a62 * a134);
  a134 = cos(a2);
  a135 = (a132 * a78);
  a136 = (a133 * a78);
  a135 = (a135 + a136);
  a135 = (a134 * a135);
  a62 = (a62 + a135);
  a64 = (a15 * a64);
  a32 = (a32 * a64);
  a69 = (a15 * a69);
  a66 = (a66 * a69);
  a32 = (a32 - a66);
  a72 = (a15 * a72);
  a67 = (a67 * a72);
  a32 = (a32 + a67);
  a104 = (a104 * a13);
  a104 = (a104 * a105);
  a32 = (a32 - a104);
  a100 = (a100 * a13);
  a100 = (a100 * a101);
  a32 = (a32 + a100);
  a107 = (a107 * a13);
  a107 = (a107 * a90);
  a32 = (a32 + a107);
  a74 = (a15 * a74);
  a32 = (a32 + a74);
  a77 = (a15 * a77);
  a32 = (a32 - a77);
  a110 = (a13 * a110);
  a32 = (a32 - a110);
  a112 = (a13 * a112);
  a32 = (a32 + a112);
  a80 = (a15 * a80);
  a32 = (a32 + a80);
  a113 = (a13 * a113);
  a32 = (a32 + a113);
  a83 = (a15 * a83);
  a32 = (a32 + a83);
  a114 = (a13 * a114);
  a32 = (a32 + a114);
  a86 = (a15 * a86);
  a32 = (a32 + a86);
  a115 = (a13 * a115);
  a32 = (a32 + a115);
  a115 = arg[0] ? arg[0][23] : 0;
  a86 = (a115 * a22);
  a58 = (a58 * a86);
  a32 = (a32 + a58);
  a58 = (a34 * a85);
  a58 = (a55 * a58);
  a32 = (a32 - a58);
  a58 = (a26 * a79);
  a58 = (a55 * a58);
  a32 = (a32 + a58);
  a58 = arg[0] ? arg[0][20] : 0;
  a114 = (a58 * a22);
  a50 = (a50 * a114);
  a32 = (a32 + a50);
  a50 = (a34 * a63);
  a50 = (a47 * a50);
  a32 = (a32 - a50);
  a50 = (a26 * a9);
  a50 = (a47 * a50);
  a32 = (a32 + a50);
  a50 = arg[0] ? arg[0][17] : 0;
  a83 = (a50 * a22);
  a42 = (a42 * a83);
  a32 = (a32 + a42);
  a42 = (a34 * a125);
  a42 = (a39 * a42);
  a32 = (a32 - a42);
  a42 = (a26 * a127);
  a42 = (a39 * a42);
  a32 = (a32 + a42);
  a42 = arg[0] ? arg[0][14] : 0;
  a113 = (a42 * a22);
  a33 = (a33 * a113);
  a32 = (a32 + a33);
  a26 = (a26 * a129);
  a26 = (a27 * a26);
  a32 = (a32 + a26);
  a34 = (a34 * a131);
  a34 = (a27 * a34);
  a32 = (a32 - a34);
  a34 = (a29 * a13);
  a26 = (a61 * a15);
  a34 = (a34 + a26);
  a34 = (a34 / a3);
  a26 = (a34 / a3);
  a33 = (a26 * a94);
  a32 = (a32 + a33);
  a33 = (a65 * a3);
  a80 = (a133 * a97);
  a33 = (a33 + a80);
  a80 = (a132 * a97);
  a33 = (a33 + a80);
  a33 = (a33 / a3);
  a80 = (a33 / a3);
  a112 = (a80 * a21);
  a32 = (a32 + a112);
  a112 = (a65 * a98);
  a32 = (a32 - a112);
  a112 = cos(a2);
  a32 = (a32 * a112);
  a2 = sin(a2);
  a34 = (a34 / a3);
  a34 = (a34 * a14);
  a34 = (a34 / a3);
  a26 = (a26 / a3);
  a26 = (a26 * a14);
  a34 = (a34 + a26);
  a34 = (a94 * a34);
  a133 = (a133 * a11);
  a26 = (a65 * a14);
  a133 = (a133 - a26);
  a132 = (a132 * a11);
  a133 = (a133 + a132);
  a133 = (a133 / a3);
  a33 = (a33 / a3);
  a33 = (a33 * a14);
  a133 = (a133 + a33);
  a133 = (a133 / a3);
  a80 = (a80 / a3);
  a80 = (a80 * a14);
  a133 = (a133 + a80);
  a133 = (a21 * a133);
  a34 = (a34 + a133);
  a78 = (a65 * a78);
  a34 = (a34 - a78);
  a34 = (a2 * a34);
  a32 = (a32 + a34);
  a62 = (a62 + a32);
  a62 = (-a62);
  if (res[0] != 0) res[0][35] = a62;
  a62 = cos(a25);
  a32 = (a86 * a14);
  a34 = (a57 * a32);
  a56 = (a79 * a56);
  a34 = (a34 - a56);
  a56 = (a114 * a14);
  a78 = (a49 * a56);
  a34 = (a34 + a78);
  a48 = (a9 * a48);
  a34 = (a34 - a48);
  a48 = (a83 * a14);
  a78 = (a41 * a48);
  a34 = (a34 + a78);
  a40 = (a127 * a40);
  a34 = (a34 - a40);
  a40 = (a113 * a14);
  a78 = (a30 * a40);
  a34 = (a34 + a78);
  a28 = (a129 * a28);
  a34 = (a34 - a28);
  a62 = (a62 * a34);
  a34 = sin(a25);
  a59 = (a85 * a59);
  a32 = (a53 * a32);
  a59 = (a59 - a32);
  a56 = (a45 * a56);
  a59 = (a59 - a56);
  a51 = (a63 * a51);
  a59 = (a59 + a51);
  a48 = (a37 * a48);
  a59 = (a59 - a48);
  a43 = (a125 * a43);
  a59 = (a59 + a43);
  a40 = (a23 * a40);
  a59 = (a59 - a40);
  a35 = (a131 * a35);
  a59 = (a59 + a35);
  a34 = (a34 * a59);
  a62 = (a62 - a34);
  if (res[0] != 0) res[0][36] = a62;
  a62 = (a69 * a14);
  a62 = (a29 * a62);
  a34 = (a72 * a14);
  a34 = (a61 * a34);
  a62 = (a62 - a34);
  a120 = (a13 * a120);
  a120 = (a29 * a120);
  a62 = (a62 - a120);
  a121 = (a13 * a121);
  a121 = (a61 * a121);
  a62 = (a62 - a121);
  a121 = (a84 * a11);
  a121 = (a29 * a121);
  a62 = (a62 + a121);
  a11 = (a95 * a11);
  a11 = (a61 * a11);
  a62 = (a62 - a11);
  a11 = (a15 * a14);
  a11 = (a76 * a11);
  a14 = (a13 * a14);
  a14 = (a111 * a14);
  a11 = (a11 - a14);
  a75 = (a75 * a11);
  a62 = (a62 + a75);
  a62 = (a62 + a12);
  if (res[0] != 0) res[0][37] = a62;
  a62 = (a4 * a29);
  a12 = (a95 * a65);
  a62 = (a62 - a12);
  a81 = (a71 * a81);
  a62 = (a62 + a81);
  a17 = (a17 * a0);
  a62 = (a62 + a17);
  a17 = (a98 * a15);
  a62 = (a62 - a17);
  a62 = (a134 * a62);
  a17 = (a64 * a29);
  a81 = (a72 * a65);
  a17 = (a17 + a81);
  a29 = (a13 * a29);
  a29 = (a105 * a29);
  a17 = (a17 - a29);
  a29 = (a13 * a65);
  a90 = (a90 * a29);
  a17 = (a17 + a90);
  a8 = (a13 * a8);
  a17 = (a17 - a8);
  a6 = (a15 * a6);
  a17 = (a17 + a6);
  a5 = (a13 * a5);
  a17 = (a17 + a5);
  a5 = (a15 / a3);
  a5 = (a5 / a3);
  a5 = (a94 * a5);
  a17 = (a17 + a5);
  a5 = (a97 * a15);
  a5 = (a5 / a3);
  a5 = (a5 / a3);
  a5 = (a21 * a5);
  a17 = (a17 + a5);
  a17 = (a2 * a17);
  a62 = (a62 - a17);
  if (res[0] != 0) res[0][38] = a62;
  a62 = (a4 * a61);
  a17 = (a84 * a65);
  a62 = (a62 + a17);
  a99 = (a103 * a99);
  a62 = (a62 + a99);
  a19 = (a19 * a0);
  a62 = (a62 + a19);
  a98 = (a98 * a13);
  a62 = (a62 - a98);
  a134 = (a134 * a62);
  a62 = (a64 * a61);
  a98 = (a69 * a65);
  a62 = (a62 - a98);
  a61 = (a13 * a61);
  a105 = (a105 * a61);
  a62 = (a62 - a105);
  a65 = (a13 * a65);
  a101 = (a101 * a65);
  a62 = (a62 + a101);
  a122 = (a15 * a122);
  a62 = (a62 + a122);
  a123 = (a15 * a123);
  a62 = (a62 + a123);
  a96 = (a13 * a96);
  a62 = (a62 + a96);
  a96 = (a13 / a3);
  a96 = (a96 / a3);
  a94 = (a94 * a96);
  a62 = (a62 + a94);
  a94 = (a97 * a13);
  a94 = (a94 / a3);
  a94 = (a94 / a3);
  a21 = (a21 * a94);
  a62 = (a62 + a21);
  a2 = (a2 * a62);
  a134 = (a134 - a2);
  if (res[0] != 0) res[0][39] = a134;
  a10 = (a10 * a31);
  a10 = (a55 * a10);
  a115 = (a115 * a31);
  a134 = (a53 * a115);
  a10 = (a10 - a134);
  a58 = (a58 * a31);
  a134 = (a45 * a58);
  a10 = (a10 - a134);
  a87 = (a87 * a31);
  a87 = (a47 * a87);
  a10 = (a10 + a87);
  a50 = (a50 * a31);
  a87 = (a37 * a50);
  a10 = (a10 - a87);
  a124 = (a124 * a31);
  a124 = (a39 * a124);
  a10 = (a10 + a124);
  a130 = (a130 * a31);
  a130 = (a27 * a130);
  a10 = (a10 + a130);
  a42 = (a42 * a31);
  a130 = (a23 * a42);
  a10 = (a10 - a130);
  a86 = (a3 * a86);
  a130 = (a57 * a86);
  a10 = (a10 - a130);
  a54 = (a54 * a79);
  a10 = (a10 + a54);
  a114 = (a3 * a114);
  a54 = (a49 * a114);
  a10 = (a10 - a54);
  a46 = (a46 * a9);
  a10 = (a10 + a46);
  a83 = (a3 * a83);
  a46 = (a41 * a83);
  a10 = (a10 - a46);
  a38 = (a38 * a127);
  a10 = (a10 + a38);
  a113 = (a3 * a113);
  a38 = (a30 * a113);
  a10 = (a10 - a38);
  a24 = (a24 * a129);
  a10 = (a10 + a24);
  a24 = sin(a25);
  a10 = (a10 * a24);
  a82 = (a82 * a31);
  a55 = (a55 * a82);
  a57 = (a57 * a115);
  a55 = (a55 - a57);
  a49 = (a49 * a58);
  a55 = (a55 - a49);
  a70 = (a70 * a31);
  a47 = (a47 * a70);
  a55 = (a55 + a47);
  a41 = (a41 * a50);
  a55 = (a55 - a41);
  a126 = (a126 * a31);
  a39 = (a39 * a126);
  a55 = (a55 + a39);
  a128 = (a128 * a31);
  a27 = (a27 * a128);
  a55 = (a55 + a27);
  a30 = (a30 * a42);
  a55 = (a55 - a30);
  a53 = (a53 * a86);
  a55 = (a55 + a53);
  a60 = (a60 * a85);
  a55 = (a55 - a60);
  a45 = (a45 * a114);
  a55 = (a55 + a45);
  a52 = (a52 * a63);
  a55 = (a55 - a52);
  a37 = (a37 * a83);
  a55 = (a55 + a37);
  a44 = (a44 * a125);
  a55 = (a55 - a44);
  a23 = (a23 * a113);
  a55 = (a55 + a23);
  a36 = (a36 * a131);
  a55 = (a55 - a36);
  a25 = cos(a25);
  a55 = (a55 * a25);
  a10 = (a10 + a55);
  a10 = (-a10);
  if (res[0] != 0) res[0][40] = a10;
  a10 = 2.;
  a55 = (a13 * a3);
  a55 = (a55 * a111);
  a111 = (a3 * a15);
  a111 = (a111 * a76);
  a55 = (a55 - a111);
  a93 = (a15 * a93);
  a55 = (a55 - a93);
  a119 = (a13 * a119);
  a55 = (a55 + a119);
  a55 = (a10 * a55);
  if (res[0] != 0) res[0][41] = a55;
  a72 = (a3 * a72);
  a108 = (a13 * a108);
  a72 = (a72 + a108);
  a95 = (a97 * a95);
  a72 = (a72 - a95);
  a91 = (a15 * a91);
  a72 = (a72 - a91);
  a118 = (a13 * a118);
  a72 = (a72 + a118);
  a7 = (a7 * a22);
  a72 = (a72 + a7);
  if (res[0] != 0) res[0][42] = a72;
  a102 = (a13 * a102);
  a69 = (a3 * a69);
  a102 = (a102 - a69);
  a84 = (a97 * a84);
  a102 = (a102 + a84);
  a89 = (a15 * a89);
  a102 = (a102 + a89);
  a68 = (a13 * a68);
  a102 = (a102 - a68);
  a1 = (a1 * a22);
  a102 = (a102 - a1);
  if (res[0] != 0) res[0][43] = a102;
  a71 = (a97 * a71);
  a102 = (a13 * a3);
  a102 = (a102 * a109);
  a71 = (a71 - a102);
  a92 = (a15 * a92);
  a71 = (a71 + a92);
  a71 = (a10 * a71);
  a92 = (a18 * a22);
  a71 = (a71 + a92);
  a92 = (a18 * a22);
  a71 = (a71 + a92);
  if (res[0] != 0) res[0][44] = a71;
  a64 = (a3 * a64);
  a106 = (a13 * a106);
  a64 = (a64 - a106);
  a4 = (a97 * a4);
  a64 = (a64 + a4);
  a88 = (a15 * a88);
  a64 = (a64 + a88);
  a117 = (a13 * a117);
  a64 = (a64 + a117);
  a16 = (a16 * a22);
  a64 = (a64 - a16);
  a20 = (a20 * a22);
  a64 = (a64 + a20);
  if (res[0] != 0) res[0][45] = a64;
  a3 = (a3 * a15);
  a3 = (a3 * a73);
  a97 = (a97 * a103);
  a3 = (a3 + a97);
  a13 = (a13 * a116);
  a3 = (a3 + a13);
  a10 = (a10 * a3);
  a3 = (a18 * a22);
  a10 = (a10 - a3);
  a18 = (a18 * a22);
  a10 = (a10 - a18);
  if (res[0] != 0) res[0][46] = a10;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_go1(const casadi_real** arg,
                                                    casadi_real** res,
                                                    casadi_int* iw,
                                                    casadi_real* w, int mem) {
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_go1_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_go1_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_go1_free_mem(int mem) {}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_go1_checkout(void) { return 0; }

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_go1_release(int mem) {}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_go1_incref(void) {}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_go1_decref(void) {}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_go1_n_in(void) {
  return 3;
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_go1_n_out(void) {
  return 1;
}

extern "C" CASADI_SYMBOL_EXPORT casadi_real
eval_hess_g_go1_default_in(casadi_int i) {
  switch (i) {
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_go1_name_in(
    casadi_int i) {
  switch (i) {
    case 0:
      return "w";
    case 1:
      return "lambda";
    case 2:
      return "p";
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_go1_name_out(
    casadi_int i) {
  switch (i) {
    case 0:
      return "hess_g";
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_go1_sparsity_in(
    casadi_int i) {
  switch (i) {
    case 0:
      return casadi_s0;
    case 1:
      return casadi_s1;
    case 2:
      return casadi_s2;
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_go1_sparsity_out(
    casadi_int i) {
  switch (i) {
    case 0:
      return casadi_s3;
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_go1_work(casadi_int* sz_arg,
                                                         casadi_int* sz_res,
                                                         casadi_int* sz_iw,
                                                         casadi_int* sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}
