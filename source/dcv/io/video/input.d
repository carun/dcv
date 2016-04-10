﻿module dcv.io.video.input;

import std.exception : enforce;

debug {
	import std.stdio;
}

import ffmpeg.libavcodec.avcodec;
import ffmpeg.libavformat.avformat;
import ffmpeg.libavutil.avutil;
import ffmpeg.libavutil.opt;
import ffmpeg.libavutil.mem;
import ffmpeg.libswscale.swscale;
import ffmpeg.libavdevice.avdevice;
import ffmpeg.libavfilter.avfilter;

public import dcv.io.video.common;
public import dcv.io.image;


enum InputStreamType {
	INVALID,
	FILE,
	LIVE
}

class SeekFrameException : Exception {
	@safe this(size_t frame, string file = __FILE__, size_t line = __LINE__, Throwable next = null) {
		import std.conv : to;
		super("Internal error occurred while seeking frame: " ~ frame.to!string, file, line, next);
	}
}

class SeekTimeException : Exception {
	@safe this(double time, string file = __FILE__, size_t line = __LINE__, Throwable next = null) {
		import std.conv : to;
		super("Internal error occurred while seeking time: " ~ time.to!string, file, line, next);
	}
}

class ReadFrameException : Exception {
	@safe this(string msg, string file = __FILE__, size_t line = __LINE__, Throwable next = null) {
		import std.conv : to;
		super(msg, file, line, next);
	}
}


/**
 * Video streaming utility.
 * 
 * Perform FILE/output video streaming. Wraps FFmpeg library.
 */
class InputStream {
private:
	AVFormatContext* formatContext = null;
	AVStream* stream = null;
	InputStreamType type = InputStreamType.INVALID;

public:
	this() { AVStarter AV_STARTER_INSTANCE = AVStarter.instance(); }

	~this() { close(); }

	@property const {
		private auto checkStream() { if (stream is null) throw new StreamNotOpenException; }

		//! Check if stream is open.
		auto isOpen() { return formatContext !is null; }
		auto isFileStream() { return (type == InputStreamType.FILE); }
		auto isLiveStream() { return (type == InputStreamType.LIVE); }

		auto width() { checkStream(); return stream.codec.width; }
		auto height() { checkStream(); return stream.codec.height; }

		auto frameSize() { return avpicture_get_size(pixelFormat, cast(int) width, cast(int) height); }
		auto frameCount() {	
			checkStream(); 
			long fc = stream.nb_frames;
			if (fc <= 0) {
				fc = stream.nb_index_entries;
			}
			return fc;
		}
		auto streamIndex() { checkStream(); return stream.index; }
		auto frameRate() { 
			checkStream(); 
			double fps = stream.r_frame_rate.q2d;
			if (fps < float.epsilon) {
				fps = stream.avg_frame_rate.q2d;
			}
			if (fps < float.epsilon) {
				fps = 1. / stream.codec.time_base.q2d;
			}
			
			return fps;
		}

		auto duration() {
			import std.algorithm.comparison : max;
			checkStream(); 
			return stream.duration >= 0 ? stream.duration : 0;
		}
	}

	void dumpFormat() const {
		if (!isOpen)
			throw new StreamNotOpenException;
		av_dump_format(cast(AVFormatContext*) formatContext, 0, "", 0);
	}

	/**
	 * Open the video stream.
	 * 
	 * params:
	 * path = Path to the stream. 
	 * type = Stream type. 
	 */
	bool open(in string path, InputStreamType type = InputStreamType.FILE) {
		enforce(type != InputStreamType.INVALID, "Input stream type cannot be defined as invalid.");

		this.type = type;

		AVInputFormat *fmt = null;

		if (isLiveStream) {
			version (Windows) {
				fmt = av_find_input_format("dshow");
				if (fmt is null) {
					fmt = av_find_input_format("vfwcap");
				}
			}
			else version (linux) {
				fmt = av_find_input_format("v4l2");
			}
			else version (OSX) {
				fmt = av_find_input_format("avfoundation");
			}
			else {
				static assert(0, "Not supported platform");
			}
			if (fmt is null) {
				throw new OpenException("Cannot find correspoding file live format for the platform");
			}
		} 

		return openInputStreamImpl(fmt, path);
	}

	void close() {
		if (formatContext) {
			avformat_close_input(&formatContext);
			formatContext = null;
		}
		if (stream) {
			avcodec_close(stream.codec);
			stream = null;
		}
	}

	void seekFrame(size_t frame) {
		enforce(isFileStream, "Only input file streams can be seeked.");

		if (stream is null)
			throw new Exception("Stream is not open");

		if(!(frame < frameCount)) {
			throw new SeekFrameException(frame);
		}

		double frameDuration = 1. / frameRate;
		double seekSeconds = frame * frameDuration;
		int seekTarget = cast(int)(seekSeconds * (stream.time_base.den)) / (stream.time_base.num);

		if (av_seek_frame(formatContext, cast(int) streamIndex, seekTarget, AVSEEK_FLAG_ANY) < 0) {
			throw new SeekFrameException(frame);
		}
	}

	void seekTime(double time) {
		enforce(isFileStream, "Only input file streams can be seeked.");

		if (stream is null)
			throw new StreamNotOpenException;

		int seekTarget = cast(int)(time * (stream.time_base.den)) / (stream.time_base.num);

		if (av_seek_frame(formatContext, cast(int) streamIndex, seekTarget, AVSEEK_FLAG_ANY) < 0) {
			throw new SeekTimeException(time);
		}
	}

	bool readFrame(ref Image image) {
		if (isOpen) {
			return readFrameImpl(image);
		} else {
			throw new StreamNotOpenException;
		}
	}

private:

	bool readFrameImpl(ref Image image) {
		bool stat = false;

		AVPacket packet;
		av_init_packet(&packet);

		// allocating an AVFrame
		AVFrame* frame = av_frame_alloc();
		if (!frame) {
			throw new Exception("Could not allocate frame.");
		}

		scope (exit) {
			av_frame_free(&frame);
			av_free_packet(&packet);
		}

		while (av_read_frame(formatContext, &packet) >= 0) {
			int ret = 0;
			int gotFrame = 0;
			if (packet.stream_index == streamIndex) {
				while(true) {
					ret = avcodec_decode_video2(stream.codec, frame, &gotFrame, &packet);
					if (ret < 0) {
						throw new StreamException("Error decoding video frame.");
					}
					if (gotFrame)
						break;
				}
				if (gotFrame) {
					stat = true;

					if (image is null || image.byteSize != frameSize) {
						image = new Image(width, height,
							AVPixelFormat_to_ImageFormat(pixelFormat), BitDepth.BD_8);
					}

					adoptFormat(pixelFormat, frame, image.data);
					break;
				}
			}
		}
		return stat;
	}

	bool openInputStreamImpl(AVInputFormat* inputFormat, in string filepath) {
		const char* file = cast(const char*) filepath.dup.ptr;
		int streamIndex = -1;

		// open file, and allocate format context
		if (avformat_open_input(&formatContext, filepath.ptr, inputFormat, null) < 0) {
			debug writeln("Could not open stream for file: " ~ filepath);
			return false;
		}

		// retrieve stream information 
		if (avformat_find_stream_info(formatContext, null) < 0) {
			debug writeln("Could not find stream information");
			return false;
		}

		if (openCodecContext(&streamIndex, formatContext, AVMediaType.AVMEDIA_TYPE_VIDEO) >= 0) {
			stream = formatContext.streams[streamIndex];
		}

		if (!stream) {
			debug writeln("Could not find video stream.");
			return false;
		}

		return true;
	}

	
	int openCodecContext(int* stream_idx, AVFormatContext* fmt_ctx, AVMediaType type) {
		int ret;

		AVStream* st;
		AVCodecContext* dec_ctx = null;
		AVCodec* dec = null;
		AVDictionary* opts = null;

		ret = av_find_best_stream(fmt_ctx, type, -1, -1, null, 0);
		if (ret < 0) {
			debug writeln("Could not find stream in FILE file.");
			return ret;
		}
		else {
			*stream_idx = ret;
			st = fmt_ctx.streams[*stream_idx];
			/* find decoder for the stream */
			dec_ctx = st.codec;
			dec = avcodec_find_decoder(dec_ctx.codec_id);

			if (!dec) {
				debug writeln("Failed to find codec: ", av_get_media_type_string(type));
				return -1;
			}

			if ((ret = avcodec_open2(dec_ctx, dec, &opts)) < 0) {
				debug writeln("Failed to open codec: ", av_get_media_type_string(type));
				return ret;
			}
		}

		return 0;
	}

	@property AVPixelFormat pixelFormat() const {
		if (stream is null)
			throw new StreamNotOpenException;
		return convertDepricatedPixelFormat(stream.codec.pix_fmt);
	}

}
