/**
 * \file Geoid.cpp
 * \brief Implementation for GeographicLib::Geoid class
 *
 * Copyright (c) Charles Karney (2009-2020) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <AP_Geoid/Geoid.h>
#include <AP_Math/AP_Math.h>

#include <AP_HAL/AP_HAL.h>

#define MAX_LINE_SIZE 256

extern const AP_HAL::HAL& hal;

#if GEOGRAPHICLIB_GEOID_ENABLE_CACHE
#include <AP_Geoid/Utility.h>
#endif

namespace GeographicLib {

  using namespace std;

  // This is the transfer matrix for a 3rd order fit with a 12-point stencil
  // with weights
  //
  //   \x -1  0  1  2
  //   y
  //  -1   .  1  1  .
  //   0   1  2  2  1
  //   1   1  2  2  1
  //   2   .  1  1  .
  //
  // A algorithm for n-dimensional polynomial fits is described in
  //   F. H. Lesh,
  //   Multi-dimensional least-squares polynomial curve fitting,
  //   CACM 2, 29-30 (1959).
  //   https://doi.org/10.1145/368424.368443
  //
  // Here's the Maxima code to generate this matrix:
  //
  // /* The stencil and the weights */
  // xarr:[
  //     0, 1,
  // -1, 0, 1, 2,
  // -1, 0, 1, 2,
  //     0, 1]$
  // yarr:[
  //   -1,-1,
  // 0, 0, 0, 0,
  // 1, 1, 1, 1,
  //    2, 2]$
  // warr:[
  //    1, 1,
  // 1, 2, 2, 1,
  // 1, 2, 2, 1,
  //    1, 1]$
  //
  // /* [x exponent, y exponent] for cubic fit */
  // pows:[
  // [0,0],
  // [1,0],[0,1],
  // [2,0],[1,1],[0,2],
  // [3,0],[2,1],[1,2],[0,3]]$
  //
  // basisvec(x,y,pows):=map(lambda([ex],(if ex[1]=0 then 1 else x^ex[1])*
  //     (if ex[2]=0 then 1 else y^ex[2])),pows)$
  // addterm(x,y,f,w,pows):=block([a,b,bb:basisvec(x,y,pows)],
  //   a:w*(transpose(bb).bb),
  //   b:(w*f) * bb,
  //   [a,b])$
  //
  // c3row(k):=block([a,b,c,pows:pows,n],
  //   n:length(pows),
  //   a:zeromatrix(n,n),
  //   b:copylist(part(a,1)),
  //   c:[a,b],
  //   for i:1 thru length(xarr) do
  //   c:c+addterm(xarr[i],yarr[i],if i=k then 1 else 0,warr[i],pows),
  //   a:c[1],b:c[2],
  //   part(transpose( a^^-1 . transpose(b)),1))$
  // c3:[]$
  // for k:1 thru length(warr) do c3:endcons(c3row(k),c3)$
  // c3:apply(matrix,c3)$
  // c0:part(ratsimp(
  // genmatrix(yc,1,length(warr)).abs(c3).genmatrix(yd,length(pows),1)),2)$
  // c3:c0*c3$

  const int Geoid::c0_ = 240; // Common denominator
  const int Geoid::c3_[stencilsize_ * nterms_] = {
      9, -18, -88,    0,  96,   90,   0,   0, -60, -20,
     -9,  18,   8,    0, -96,   30,   0,   0,  60, -20,
      9, -88, -18,   90,  96,    0, -20, -60,   0,   0,
    186, -42, -42, -150, -96, -150,  60,  60,  60,  60,
     54, 162, -78,   30, -24,  -90, -60,  60, -60,  60,
     -9, -32,  18,   30,  24,    0,  20, -60,   0,   0,
     -9,   8,  18,   30, -96,    0, -20,  60,   0,   0,
     54, -78, 162,  -90, -24,   30,  60, -60,  60, -60,
    -54,  78,  78,   90, 144,   90, -60, -60, -60, -60,
      9,  -8, -18,  -30, -24,    0,  20,  60,   0,   0,
     -9,  18, -32,    0,  24,   30,   0,   0, -60,  20,
      9, -18,  -8,    0, -24,  -30,   0,   0,  60,  20,
  };

  // Like c3, but with the coeffs of x, x^2, and x^3 constrained to be zero.
  // Use this at the N pole so that the height in independent of the longitude
  // there.
  //
  // Here's the Maxima code to generate this matrix (continued from above).
  //
  // /* figure which terms to exclude so that fit is indep of x at y=0 */
  // mask:part(zeromatrix(1,length(pows)),1)+1$
  // for i:1 thru length(pows) do
  // if pows[i][1]>0 and pows[i][2]=0 then mask[i]:0$
  //
  // /* Same as c3row but with masked pows. */
  // c3nrow(k):=block([a,b,c,powsa:[],n,d,e],
  //   for i:1 thru length(mask) do if mask[i]>0 then
  //   powsa:endcons(pows[i],powsa),
  //   n:length(powsa),
  //   a:zeromatrix(n,n),
  //   b:copylist(part(a,1)),
  //   c:[a,b],
  //   for i:1 thru length(xarr) do
  //   c:c+addterm(xarr[i],yarr[i],if i=k then 1 else 0,warr[i],powsa),
  //   a:c[1],b:c[2],
  //   d:part(transpose( a^^-1 . transpose(b)),1),
  //   e:[],
  //   for i:1 thru length(mask) do
  //   if mask[i]>0 then (e:endcons(first(d),e),d:rest(d)) else e:endcons(0,e),
  //   e)$
  // c3n:[]$
  // for k:1 thru length(warr) do c3n:endcons(c3nrow(k),c3n)$
  // c3n:apply(matrix,c3n)$
  // c0n:part(ratsimp(
  //    genmatrix(yc,1,length(warr)).abs(c3n).genmatrix(yd,length(pows),1)),2)$
  // c3n:c0n*c3n$

  const int Geoid::c0n_ = 372; // Common denominator
  const int Geoid::c3n_[stencilsize_ * nterms_] = {
      0, 0, -131, 0,  138,  144, 0,   0, -102, -31,
      0, 0,    7, 0, -138,   42, 0,   0,  102, -31,
     62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31,
    124, 0,  -62, 0,    0, -124, 0,   0,    0,  62,
    124, 0,  -62, 0,    0, -124, 0,   0,    0,  62,
     62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31,
      0, 0,   45, 0, -183,   -9, 0,  93,   18,   0,
      0, 0,  216, 0,   33,   87, 0, -93,   12, -93,
      0, 0,  156, 0,  153,   99, 0, -93,  -12, -93,
      0, 0,  -45, 0,   -3,    9, 0,  93,  -18,   0,
      0, 0,  -55, 0,   48,   42, 0,   0,  -84,  31,
      0, 0,   -7, 0,  -48,  -42, 0,   0,   84,  31,
  };

  // Like c3n, but y -> 1-y so that h is independent of x at y = 1.  Use this
  // at the S pole so that the height in independent of the longitude there.
  //
  // Here's the Maxima code to generate this matrix (continued from above).
  //
  // /* Transform c3n to c3s by transforming y -> 1-y */
  // vv:[
  //      v[11],v[12],
  // v[7],v[8],v[9],v[10],
  // v[3],v[4],v[5],v[6],
  //      v[1],v[2]]$
  // poly:expand(vv.(c3n/c0n).transpose(basisvec(x,1-y,pows)))$
  // c3sf[i,j]:=coeff(coeff(coeff(poly,v[i]),x,pows[j][1]),y,pows[j][2])$
  // c3s:genmatrix(c3sf,length(vv),length(pows))$
  // c0s:part(ratsimp(
  //    genmatrix(yc,1,length(warr)).abs(c3s).genmatrix(yd,length(pows),1)),2)$
  // c3s:c0s*c3s$

  const int Geoid::c0s_ = 372; // Common denominator
  const int Geoid::c3s_[stencilsize_ * nterms_] = {
     18,  -36, -122,   0,  120,  135, 0,   0,  -84, -31,
    -18,   36,   -2,   0, -120,   51, 0,   0,   84, -31,
     36, -165,  -27,  93,  147,   -9, 0, -93,   18,   0,
    210,   45, -111, -93,  -57, -192, 0,  93,   12,  93,
    162,  141,  -75, -93, -129, -180, 0,  93,  -12,  93,
    -36,  -21,   27,  93,   39,    9, 0, -93,  -18,   0,
      0,    0,   62,   0,    0,   31, 0,   0,    0, -31,
      0,    0,  124,   0,    0,   62, 0,   0,    0, -62,
      0,    0,  124,   0,    0,   62, 0,   0,    0, -62,
      0,    0,   62,   0,    0,   31, 0,   0,    0, -31,
    -18,   36,  -64,   0,   66,   51, 0,   0, -102,  31,
     18,  -36,    2,   0,  -66,  -51, 0,   0,  102,  31,
  };

  Geoid::Geoid(const char* filepath, bool cubic)
    : _filename(filepath)
    , _cubic(cubic)
    , _a( Constants::WGS84_a() )
    , _e2( (2 - Constants::WGS84_f()) * Constants::WGS84_f() )
    , _degree( Math::degree() )
    , _eps( sqrt(numeric_limits<real>::epsilon()) )
  {
    static_assert(sizeof(pixel_t) == pixel_size_, "pixel_t has the wrong size");

    if (_filename == nullptr) {
        hal.console->printf("Geoid: must have valid filename\n");
        _io_failure = true;
        return;
    }

    // geoid file_path
    _fd = AP::FS().open(_filename, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Open %s failed\n", _filename);
        _io_failure = true;
        return;
    }
    hal.console->printf("Open %s\n", _filename);

    // parse geoid file header
    char line[MAX_LINE_SIZE];

    // check format
    if (AP::FS().fgets(line, sizeof(line)-1, _fd)) {
        if (strcmp(line, "P5") != 0) {
            hal.console->printf("File not in PGM format %s\n", _filename);
            _io_failure = true;
            AP::FS().close(_fd);
            return;
        }
    }

    _offset = numeric_limits<real>::max();
    _scale = 0.0;
    _maxerror = -1.0;
    _rmserror = -1.0;

    while (AP::FS().fgets(line, sizeof(line)-1, _fd)) {
         // tokenise
        char *saveptr = nullptr;
        const char *comment_id_str = strtok_r(line, " ", &saveptr);
        if (comment_id_str == nullptr || strlen(comment_id_str) == 0) {
            continue;
        }

        // comments
        if (strcmp(comment_id_str, "#") == 0) {
            const char *key_str = strtok_r(nullptr, " ", &saveptr);
            if (key_str == nullptr || strlen(key_str) == 0) {
                continue;
            }

            if (strcmp(key_str, "Description") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    hal.console->printf("Description: %s\n", value_str);
                }
            }
            if (strcmp(key_str, "DateTime") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    hal.console->printf("DateTime: %s\n", value_str);
                }
            }
            if (strcmp(key_str, "Offset") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _offset = atof(value_str);
                    hal.console->printf("Offset: %f\n", _offset);
                }
            }
            if (strcmp(key_str, "Scale") == 0) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _scale = atof(value_str);
                    hal.console->printf("Scale: %f\n", _scale);
                }
            }
            if (strcmp(key_str, "MaxCubicError") == 0 && _cubic) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _maxerror = atof(value_str);
                    hal.console->printf("MaxCubicError: %f\n", _maxerror);
                }
            }
            if (strcmp(key_str, "MaxBilinearError") == 0 && !_cubic) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _maxerror = atof(value_str);
                    hal.console->printf("MaxBilinearError: %f\n", _maxerror);
                }
            }
            if (strcmp(key_str, "RMSCubicError") == 0 && _cubic) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _rmserror = atof(value_str);
                    hal.console->printf("RMSCubicError: %f\n", _rmserror);
                }
            }
            if (strcmp(key_str, "RMSBilinearError") == 0 && !_cubic) {
                const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
                if (value_str != nullptr || strlen(value_str) > 0) {
                    _rmserror = atof(value_str);
                    hal.console->printf("RMSBilinearError: %f\n", _rmserror);
                }
            }
        } else {
            // read raster size
            _width = atoi(comment_id_str);
            hal.console->printf("width: %d\n", _width);

            const char *value_str = strtok_r(nullptr, "\r\n", &saveptr);
            if (value_str != nullptr || strlen(value_str) > 0) {
                _height = atoi(value_str);
                hal.console->printf("height: %d\n", _height);
            }
            break;
        }
    }

    {
        // read maxval
        unsigned maxval = 0;
        hal.console->printf("Reading maxval...\n");
        while (AP::FS().fgets(line, sizeof(line)-1, _fd)) {
            if (strlen(line) != 0) {
                maxval = atol(line);
                hal.console->printf("maxval: %d\n", maxval);
                break;
            }
        }
        if (maxval != pixel_max_) {
            hal.console->printf("Incorrect value of maxval %s\n", _filename);
            _io_failure = true;
            AP::FS().close(_fd);
            return;
        }

        // set start of data block
        hal.console->printf("Setting start of data block...\n");
        _datastart = AP::FS().lseek(_fd, 0, SEEK_CUR);
        _swidth = uint64_t(_width);
        hal.console->printf("datastart: %llu\n", _datastart);
    }
    if (is_equal(_offset, numeric_limits<real>::max())) {
      hal.console->printf("Offset not set %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }
    if (is_zero(_scale)) {
      hal.console->printf("Scale not set %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }
    if (_scale < 0) {
      hal.console->printf("Scale must be positive %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }
    if (_height < 2 || _width < 2) {
      // Coarsest grid spacing is 180deg.
      hal.console->printf("Raster size too small %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }
    if (_width & 1) {
      // This is so that longitude grids can be extended thru the poles.
      hal.console->printf("Raster width is odd %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }
    if (!(_height & 1)) {
      // This is so that latitude grid includes the equator.
      hal.console->printf("Raster height is even %s\n", _filename);
      _io_failure = true;
      AP::FS().close(_fd);
      return;
    }

    // check file size
    int32_t file_end = AP::FS().lseek(_fd, 0, SEEK_END);
    hal.console->printf("file_end: %d\n", int(file_end));

    uint32_t sheight = _height;
    if (_datastart + pixel_size_ * _swidth * sheight != uint64_t(file_end)) {
        hal.console->printf("File has the wrong length %s\n", _filename);
        _io_failure = true;
        AP::FS().close(_fd);
        return;
    }
    hal.console->printf("file length expected: %llu\n", uint64_t(_datastart + pixel_size_ * _swidth * sheight));
    hal.console->printf("file length actual:   %d\n", int(file_end));

    _rlonres = _width / real(Math::td);
    _rlatres = (_height - 1) / real(Math::hd);

#if GEOGRAPHICLIB_GEOID_ENABLE_CACHE
    _cache = false;
#endif

    _ix = _width;
    _iy = _height;
  }

  bool Geoid::height(real lat, real lon, real& h_out) const {
    lat = Math::LatFix(lat);
    if (isnan(lat) || isnan(lon)) {
      h_out = Math::NaN();
      return true;
    }
    lon = Math::AngNormalize(lon);
    real
      fx =  lon * _rlonres,
      fy = -lat * _rlatres;
    int
      ix = int(floor(fx)),
      iy = min((_height - 1)/2 - 1, int(floor(fy)));
    fx -= ix;
    fy -= iy;
    iy += (_height - 1)/2;
    ix += ix < 0 ? _width : (ix >= _width ? -_width : 0);
    real v00 = 0, v01 = 0, v10 = 0, v11 = 0;
    real t[nterms_];

    if (!(ix == _ix && iy == _iy)) {
      if (!_cubic) {
        bool read_ok = true;
        read_ok &= rawval(ix    , iy    , v00);
        read_ok &= rawval(ix + 1, iy    , v01);
        read_ok &= rawval(ix    , iy + 1, v10);
        read_ok &= rawval(ix + 1, iy + 1, v11);
        if (!read_ok) {
          h_out = Math::NaN();
          return false;
        }
      } else {
        real v[stencilsize_];
        int k = 0;
        bool read_ok = true;
        read_ok &= rawval(ix    , iy - 1, v[k++]);
        read_ok &= rawval(ix + 1, iy - 1, v[k++]);
        read_ok &= rawval(ix - 1, iy    , v[k++]);
        read_ok &= rawval(ix    , iy    , v[k++]);
        read_ok &= rawval(ix + 1, iy    , v[k++]);
        read_ok &= rawval(ix + 2, iy    , v[k++]);
        read_ok &= rawval(ix - 1, iy + 1, v[k++]);
        read_ok &= rawval(ix    , iy + 1, v[k++]);
        read_ok &= rawval(ix + 1, iy + 1, v[k++]);
        read_ok &= rawval(ix + 2, iy + 1, v[k++]);
        read_ok &= rawval(ix    , iy + 2, v[k++]);
        read_ok &= rawval(ix + 1, iy + 2, v[k++]);
        if (!read_ok) {
          h_out = Math::NaN();
          return false;
        }

        const int* c3x = iy == 0 ? c3n_ : (iy == _height - 2 ? c3s_ : c3_);
        int c0x = iy == 0 ? c0n_ : (iy == _height - 2 ? c0s_ : c0_);
        for (unsigned i = 0; i < nterms_; ++i) {
          t[i] = 0;
          for (unsigned j = 0; j < stencilsize_; ++j)
            t[i] += v[j] * c3x[nterms_ * j + i];
          t[i] /= c0x;
        }
      }
    } else { // same cell; used cached coefficients
      if (!_cubic) {
        v00 = _v00;
        v01 = _v01;
        v10 = _v10;
        v11 = _v11;
      } else
        copy(_t, _t + nterms_, t);
    }
    if (!_cubic) {
      real
        a = (1 - fx) * v00 + fx * v01,
        b = (1 - fx) * v10 + fx * v11,
        c = (1 - fy) * a + fy * b,
        h = _offset + _scale * c;
      {
        _ix = ix;
        _iy = iy;
        _v00 = v00;
        _v01 = v01;
        _v10 = v10;
        _v11 = v11;
      }
      h_out = h;
      return true;
    } else {
      real h = t[0] + fx * (t[1] + fx * (t[3] + fx * t[6])) +
        fy * (t[2] + fx * (t[4] + fx * t[7]) +
             fy * (t[5] + fx * t[8] + fy * t[9]));
      h = _offset + _scale * h;
      {
        _ix = ix;
        _iy = iy;
        copy(t, t + nterms_, _t);
      }
      h_out = h;
      return true;
    }
  }

#if GEOGRAPHICLIB_GEOID_ENABLE_CACHE
  void Geoid::CacheClear() const {
    {
      _cache = false;
      try {
        _data.clear();
        // Use swap to release memory back to system
        vector< vector<pixel_t> >().swap(_data);
      }
      catch (const exception&) {
      }
    }
  }

  bool Geoid::CacheArea(real south, real west, real north, real east) const {
    if (south > north) {
      CacheClear();
      return true;
    }
    south = Math::LatFix(south);
    north = Math::LatFix(north);
    west = Math::AngNormalize(west); // west in [-180, 180)
    east = Math::AngNormalize(east);
    if (east <= west)
      east += Math::td;         // east - west in (0, 360]
    int
      iw = int(floor(west * _rlonres)),
      ie = int(floor(east * _rlonres)),
      in = int(floor(-north * _rlatres)) + (_height - 1)/2,
      is = int(floor(-south * _rlatres)) + (_height - 1)/2;
    in = max(0, min(_height - 2, in));
    is = max(0, min(_height - 2, is));
    is += 1;
    ie += 1;
    if (_cubic) {
      in -= 1;
      is += 1;
      iw -= 1;
      ie += 1;
    }
    if (ie - iw >= _width - 1) {
      // Include entire longitude range
      iw = 0;
      ie = _width - 1;
    } else {
      ie += iw < 0 ? _width : (iw >= _width ? -_width : 0);
      iw += iw < 0 ? _width : (iw >= _width ? -_width : 0);
    }
    int oysize = int(_data.size());
    _xsize = ie - iw + 1;
    _ysize = is - in + 1;
    _xoffset = iw;
    _yoffset = in;

    try {
      _data.resize(_ysize, vector<pixel_t>(_xsize));
      for (int iy = min(oysize, _ysize); iy--;)
        _data[iy].resize(_xsize);
    }
    catch (const bad_alloc&) {
      CacheClear();
      hal.console->printf("Insufficient memory for caching %s\n", _filename.c_str());
      return false;
    }

    try {
      for (int iy = in; iy <= is; ++iy) {
        int iy1 = iy, iw1 = iw;
        if (iy < 0 || iy >= _height) {
          // Allow points "beyond" the poles to support interpolation
          iy1 = iy1 < 0 ? -iy1 : 2 * (_height - 1) - iy1;
          iw1 += _width/2;
          if (iw1 >= _width)
            iw1 -= _width;
        }
        int xs1 = min(_width - iw1, _xsize);
        filepos(iw1, iy1);
        Utility::readarray<pixel_t, pixel_t, true>
          (_file, &(_data[iy - in][0]), xs1);
        if (xs1 < _xsize) {
          // Wrap around longitude = 0
          filepos(0, iy1);
          Utility::readarray<pixel_t, pixel_t, true>
            (_file, &(_data[iy - in][xs1]), _xsize - xs1);
        }
      }
      _cache = true;
    }
    catch (const exception& e) {
      CacheClear();
      hal.console->printf("Error filling cache %s", e.what());
      return false;
    }
    return true;
  }
#endif

} // namespace GeographicLib
