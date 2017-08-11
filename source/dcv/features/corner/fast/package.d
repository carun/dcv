/**
   Module implements FAST<sup>[1]</sup> corner detector algorithm - Features from accelerated segment test (FAST) algorithm
   discovered and developed by Edward Rosten and Tom Drummond.

   This package offers D class interface to machine generated C code, adopted
   to D, which is located originally on $(LINK3 https://github.com/edrosten/fast-C-src, Edward Rosten's github).

   Copyright: Copyright (c) 2006, 2008 Edward Rosten, Relja Ljubobratovic 2016

   Authors: Edward Rosten, Relja Ljubobratovic

   License: $(LINK3 http://www.boost.org/LICENSE_1_0.txt, Boost Software License - Version 1.0).

   1. Edward Rosten, Tom Drummond (2005). "Fusing points and lines for high performance tracking", IEEE International Conference on Computer Vision 2: 1508–1511.
 */
module dcv.features.corner.fast;

import mir.ndslice;

import dcv.features.corner.fast.base:xy;
import dcv.features.corner.fast.fast_9;
import dcv.features.corner.fast.fast_10;
import dcv.features.corner.fast.fast_11;
import dcv.features.corner.fast.fast_12;
import dcv.features.corner.fast.nonmax;

import dcv.features.common;


/// Pixel neighborhood type, described in the paper.
enum FASTType
{
    t9,
    t10,
    t11,
    t12,
}

struct FASTProperties
{
    /// corner threshold
    uint threshold = 100;
    /// should non-max suporession be performed.
    bool suppressnm = true;
    /// type of the detector
    FASTType type = FASTType.t9;
}


auto fastDetector(FASTProperties properties)
{
    return FASTDetector(properties);
}

/**
   FAST corner detector utility.
 */
struct FASTDetector
{
    mixin BaseDetector;

    @disable this();

    this(FASTProperties properties)
    {
        this.properties = properties;
    }

private:
    /**
       Detect features for given image.

       Properties.
       image = Input image where corners are to be found. Only 8-bit mono image is supported as this time.
       count = How many corners are to be found.

       Returns:
       Array of found feature points.
     */
    Feature[] evaluateImpl(size_t[] packs, T)
    (
        Slice!(Contiguous, packs, const(T)*) image
    )
    {
        import core.stdc.stdlib : free;
        enum errMsg = "FAST detector is only available for 8bit mono images.";

        static assert(is (T == ubyte), errMsg);

        Feature[] features;            // output features
        int       featureCount = 0;    // feature count - internal use
        int*      featureScore = null; // feature score intenal use
        xy*       xyFeatures   = null; // xy (feature coordinate) array - internal use

        const auto imdata      = image.iterator;
        const auto imwidth     = cast(int)image.length !1;
        const auto imheight    = cast(int)image.length !0;
        const auto imrowstride = imwidth;

        scope (exit)
        {
            free(featureScore);
            free(xyFeatures);
        }

        xy* function(const ubyte*, int, int, int, int, int*) detectFunc;
        int* function(const ubyte* i, int stride, xy* corners, int num_corners, int b) scoreFunc;

        final switch (properties.type)
        {
        case FASTType.t9:
            detectFunc = &fast9_detect;
            scoreFunc  = &fast9_score;
            break;
        case FASTType.t10:
            detectFunc = &fast10_detect;
            scoreFunc  = &fast10_score;
            break;
        case FASTType.t11:
            detectFunc = &fast11_detect;
            scoreFunc  = &fast11_score;
            break;
        case FASTType.t12:
            detectFunc = &fast12_detect;
            scoreFunc  = &fast12_score;
        }

        xyFeatures = detectFunc(imdata, imwidth, imheight, imrowstride, properties.threshold, &featureCount);

        if (properties.suppressnm)
        {
            import core.stdc.stdlib : free;

            int nonMaxFeatureCount = 0;
            xy* xySuppressed       = null;

            featureScore = scoreFunc(imdata, imrowstride, xyFeatures, featureCount, cast(int)properties.threshold);
            xySuppressed = nonmax_suppression(xyFeatures, featureScore, featureCount, &nonMaxFeatureCount);

            // free the score before the suppresion
            free(featureScore);
            featureScore = null;

            // assign new feature count after the suppression
            featureCount = nonMaxFeatureCount;

            // free previous features and assign new ones.
            free(xyFeatures);
            xyFeatures = xySuppressed;
        }

        featureScore = scoreFunc(imdata, imrowstride, xyFeatures, featureCount, cast(int)properties.threshold);

        // Convert FAST results to features
        features.length = featureCount;
        foreach (i, ref feature; features)
        {
            feature.x      = xyFeatures[i].x;
            feature.y      = xyFeatures[i].y;
            feature.width  = 16.;
            feature.height = 16.;
            feature.octave = 0;
            feature.score  = featureScore[i];
        }

        return features;
    }

    FASTProperties properties;
}

