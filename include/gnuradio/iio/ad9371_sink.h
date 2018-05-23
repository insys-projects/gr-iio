/* -*- c++ -*- */
/* 
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_IIO_AD9371_SINK_H
#define INCLUDED_IIO_AD9371_SINK_H

#include <gnuradio/iio/api.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/sync_block.h>

#include "device_sink.h"

namespace gr {
  namespace iio {

    class IIO_API ad9371_sink : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<ad9371_sink> sptr;

      static sptr make(const std::string &uri, unsigned long long frequency,
		    unsigned long samplerate,
		    unsigned long bandwidth,
		    bool ch1_en, bool ch2_en, bool ch3_en, bool ch4_en,
		    unsigned long buffer_size, bool cyclic,
		    const char *rf_port_select, double attenuation1,
		    double attenuation2, const char *filter = "",
		    bool auto_filter = true, const char *params = "");

      static sptr make_from(struct iio_context *ctx,
		    unsigned long long frequency, unsigned long samplerate,
		    unsigned long bandwidth,
		    bool ch1_en, bool ch2_en, bool ch3_en, bool ch4_en,
		    unsigned long buffer_size, bool cyclic,
		    const char *rf_port_select, double attenuation1,
		    double attenuation2, const char *filter = "",
		    bool auto_filter = true, const char *params = "");

      virtual void set_params(unsigned long long frequency,
		      unsigned long samplerate, unsigned long bandwidth,
		      const char *rf_port_select, double attenuation1,
		      double attenuation2, const char *filter = "",
		      bool auto_filter = true, const char *params = "") = 0;
    };

    class IIO_API ad9371_sink_f32c : virtual public gr::hier_block2
    {
    public:
      typedef boost::shared_ptr<ad9371_sink_f32c> sptr;

      static sptr make(const std::string &uri, unsigned long long frequency,
		    unsigned long samplerate,
		    unsigned long bandwidth, bool rx1_en, bool rx2_en,
		    unsigned long buffer_size, bool cyclic,
		    const char *rf_port_select, double attenuation1,
		    double attenuation2, const char *filter = "",
		    bool auto_filter = true, const char *params = "")
      {
	      ad9371_sink::sptr block = ad9371_sink::make(uri, frequency,
			      samplerate, bandwidth, rx1_en,
			      rx1_en, rx2_en, rx2_en, buffer_size, cyclic,
			      rf_port_select, attenuation1, attenuation2,
				  filter, auto_filter, params);

	      return gnuradio::get_initial_sptr(
			      new ad9371_sink_f32c(rx1_en, rx2_en, block));
      }

      void set_params(unsigned long long frequency,
		      unsigned long samplerate, unsigned long bandwidth,
		      const char *rf_port_select, double attenuation1,
		      double attenuation2, const char *filter = "",
		      bool auto_filter = true, const char *params = "")
      {
	      ad9371_block->set_params(frequency, samplerate, bandwidth,
			      rf_port_select, attenuation1, attenuation2,
				  filter, auto_filter, params);
      }

    private:
      ad9371_sink::sptr ad9371_block;

    protected:
      explicit ad9371_sink_f32c(bool rx1_en, bool rx2_en,
		      ad9371_sink::sptr block);
    };

  } // namespace iio
} // namespace gr

#endif /* INCLUDED_IIO_AD9371_SINK_H */

