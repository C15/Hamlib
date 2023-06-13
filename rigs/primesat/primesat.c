/*
 *  Hamlib Primesat backend - main file
 *  Contibuted by Manuel Santos
 *  
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

// cppcheck-suppress *
#include <stdint.h>
// cppcheck-suppress *
#include <stdio.h>
// cppcheck-suppress *
#include <stdlib.h>
// cppcheck-suppress *
#include <string.h>  /* String function definitions */
// cppcheck-suppress *
#include <unistd.h>  /* UNIX standard function definitions */
// cppcheck-suppress *
#include <time.h>

#include "hamlib/rig.h"
#include "serial.h"
#include "parallel.h"
#include "cm108.h"
#include "gpio.h"
#include "misc.h"
#include "tones.h"
#include "idx_builtin.h"
#include "register.h"

#include "primesat.h"


#define NB_CHAN 22      /* see caps->chan_list */

#define CMDSLEEP 20*1000  /* ms for each command */

struct primesat_priv_data
{
    /* current vfo already in rig_state ? */
    vfo_t curr_vfo;
    vfo_t last_vfo; /* VFO A or VFO B, when in MEM mode */

    split_t split;
    vfo_t tx_vfo;
    int bank;
    value_t parms[RIG_SETTING_MAX];

    channel_t *curr;    /* points to vfo_a, vfo_b or mem[] */

    // we're trying to emulate all sorts of vfo possibilities so this looks redundant
    channel_t vfo_a;
    channel_t vfo_b;
    channel_t vfo_main;
    channel_t vfo_sub;

    struct ext_list *ext_funcs;
    struct ext_list *ext_parms;

    char *magic_conf;
    int static_data;

    //freq_t freq_vfoa;
    //freq_t freq_vfob;
};

/* levels pertain to each VFO */
static const struct confparams primesat_ext_levels[] =
{
    {
        TOK_EL_MAGICLEVEL, "MGL", "Magic level", "Magic level, as an example",
        NULL, RIG_CONF_NUMERIC, { .n = { 0, 1, .001 } }
    },
    {
        TOK_EL_MAGICFUNC, "MGF", "Magic func", "Magic function, as an example",
        NULL, RIG_CONF_CHECKBUTTON
    },
    {
        TOK_EL_MAGICOP, "MGO", "Magic Op", "Magic Op, as an example",
        NULL, RIG_CONF_BUTTON
    },
    {
        TOK_EL_MAGICCOMBO, "MGC", "Magic combo", "Magic combo, as an example",
        "VALUE1", RIG_CONF_COMBO, { .c = { .combostr = { "VALUE1", "VALUE2", "NONE", NULL } } }
    },
    { RIG_CONF_END, NULL, }
};

static const struct confparams primesat_ext_funcs[] =
{
    {
        TOK_EL_MAGICEXTFUNC, "MGEF", "Magic ext func", "Magic ext function, as an example",
        NULL, RIG_CONF_CHECKBUTTON
    },
    { RIG_CONF_END, NULL, }
};

/* parms pertain to the whole rig */
static const struct confparams primesat_ext_parms[] =
{
    {
        TOK_EP_MAGICPARM, "MGP", "Magic parm", "Magic parameter, as an example",
        NULL, RIG_CONF_NUMERIC, { .n = { 0, 1, .001 } }
    },
    { RIG_CONF_END, NULL, }
};

/* cfgparams are configuration item generally used by the backend's open() method */
static const struct confparams primesat_cfg_params[] =
{
    {
        TOK_CFG_MAGICCONF, "mcfg", "Magic conf", "Magic parameter, as an example",
        "DX", RIG_CONF_STRING, { }
    },
    {
        TOK_CFG_STATIC_DATA, "static_data", "Static data", "Output only static data, no randomization of meter values",
        "0", RIG_CONF_CHECKBUTTON, { }
    },
    { RIG_CONF_END, NULL, }
};

/********************************************************************/

static void copy_chan(channel_t *dest, const channel_t *src)
{
    struct ext_list *saved_ext_levels;
    int i;

    /* TODO: ext_levels[] of different sizes */

    for (i = 0; !RIG_IS_EXT_END(src->ext_levels[i]) &&
            !RIG_IS_EXT_END(dest->ext_levels[i]); i++)
    {
        dest->ext_levels[i] = src->ext_levels[i];
    }

    saved_ext_levels = dest->ext_levels;
    memcpy(dest, src, sizeof(channel_t));
    dest->ext_levels = saved_ext_levels;
}

static int primesat_init(RIG *rig)
{
    struct primesat_priv_data *priv;
    int i;

    ENTERFUNC;
    priv = (struct primesat_priv_data *)calloc(1, sizeof(struct primesat_priv_data));
    if (!priv)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    rig->state.priv = (void *)priv;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    RETURNFUNC(RIG_OK);
}

static int primesat_cleanup(RIG *rig)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int i;

    ENTERFUNC;

    free(priv->vfo_a.ext_levels);
    free(priv->vfo_b.ext_levels);
    free(priv->ext_funcs);
    free(priv->ext_parms);
    free(priv->magic_conf);

    if (rig->state.priv)
    {
        free(rig->state.priv);
    }

    rig->state.priv = NULL;

    RETURNFUNC(RIG_OK);
}

static int primesat_open(RIG *rig)
{
    ENTERFUNC;

    if (rig->caps->rig_model == RIG_MODEL_PRIMECONTROLLER)
    {
        // then we emulate a rig without set_vfo or get_vfo
        rig_debug(RIG_DEBUG_VERBOSE, "%s: Emulating rig without get_vfo or set_vfo\n",
                  __func__);
        rig->caps->set_vfo = NULL;
        rig->caps->get_vfo = NULL;
    }

    usleep(CMDSLEEP);

    RETURNFUNC(RIG_OK);
}

static int primesat_close(RIG *rig)
{
    ENTERFUNC;

    usleep(CMDSLEEP);

    RETURNFUNC(RIG_OK);
}

static int primesat_set_conf(RIG *rig, token_t token, const char *val)
{
    struct primesat_priv_data *priv;

    ENTERFUNC;
    priv = (struct primesat_priv_data *)rig->state.priv;


    RETURNFUNC(RIG_OK);
}

static int primesat_get_conf(RIG *rig, token_t token, char *val)
{
    struct primesat_priv_data *priv;

    ENTERFUNC;
    priv = (struct primesat_priv_data *)rig->state.priv;

    RETURNFUNC(RIG_OK);
}

static int primesat_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    char fstr[20];

    ENTERFUNC;

    rig_debug(RIG_DEBUG_TRACE, "%s: curr->freq=%.0f, curr->tx_freq=%.0f\n",
              __func__,
              priv->curr->freq, priv->curr->tx_freq);
    RETURNFUNC(RIG_OK);
}


static int primesat_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;

    usleep(CMDSLEEP);

    rig_debug(RIG_DEBUG_TRACE, "%s: freq=%.0f\n", __func__, *freq);
    RETURNFUNC(RIG_OK);
}


static int primesat_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    char buf[16];

    ENTERFUNC;
    usleep(CMDSLEEP);

    RETURNFUNC(RIG_OK);
}


static int primesat_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;
    usleep(CMDSLEEP);

    RETURNFUNC(RIG_OK);
}


static int primesat_set_vfo(RIG *rig, vfo_t vfo)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    RETURNFUNC(RIG_OK);
}

static int primesat_get_vfo(RIG *rig, vfo_t *vfo)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;
    usleep(CMDSLEEP);
    *vfo = priv->curr_vfo;

    RETURNFUNC(RIG_OK);
}


static int primesat_set_split_freq(RIG *rig, vfo_t vfo, freq_t tx_freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int retval;

    ENTERFUNC;

    retval = primesat_set_freq(rig, vfo, tx_freq);
    priv->curr->tx_freq = tx_freq;
    rig_debug(RIG_DEBUG_VERBOSE, "%s: priv->curr->tx_freq = %.0f\n", __func__,
              priv->curr->tx_freq);

    RETURNFUNC(retval);
}

static int primesat_get_split_freq(RIG *rig, vfo_t vfo, freq_t *tx_freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;

    *tx_freq = priv->curr->tx_freq;
    rig_debug(RIG_DEBUG_VERBOSE, "%s: priv->curr->tx_freq = %.0f\n", __func__,
              priv->curr->tx_freq);

    RETURNFUNC(RIG_OK);
}

static int primesat_set_split_mode(RIG *rig, vfo_t vfo, rmode_t tx_mode,
                                pbwidth_t tx_width)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;

    curr->tx_mode = tx_mode;

    if (RIG_PASSBAND_NOCHANGE == tx_width) { RETURNFUNC(RIG_OK); }

    curr->tx_width = tx_width;

    RETURNFUNC(RIG_OK);
}

static int primesat_get_split_mode(RIG *rig, vfo_t vfo, rmode_t *tx_mode,
                                pbwidth_t *tx_width)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;

    *tx_mode = curr->tx_mode;
    *tx_width = curr->tx_width;

    RETURNFUNC(RIG_OK);
}

static int primesat_set_split_vfo(RIG *rig, vfo_t vfo, split_t split, vfo_t tx_vfo)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    rig_debug(RIG_DEBUG_VERBOSE, "%s: split=%d, vfo=%s, tx_vfo=%s\n",
              __func__, split, rig_strvfo(vfo), rig_strvfo(tx_vfo));
    curr->split = split;
    priv->tx_vfo = tx_vfo;

    RETURNFUNC(RIG_OK);
}


static int primesat_get_split_vfo(RIG *rig, vfo_t vfo, split_t *split,
                               vfo_t *tx_vfo)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    *split = curr->split;

    RETURNFUNC(RIG_OK);
}



static int primesat_set_ts(RIG *rig, vfo_t vfo, shortfreq_t ts)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    curr->tuning_step = ts;

    RETURNFUNC(RIG_OK);
}


static int primesat_get_ts(RIG *rig, vfo_t vfo, shortfreq_t *ts)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    *ts = curr->tuning_step;

    RETURNFUNC(RIG_OK);
}


static int primesat_set_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    char lstr[64];
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;

    RETURNFUNC(RIG_OK);
}

static int primesat_get_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;

    RETURNFUNC(RIG_OK);
}


static int primesat_set_ext_func(RIG *rig, vfo_t vfo, token_t token, int status)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %d\n", __func__,
              cfp->name, status);

    RETURNFUNC(RIG_OK);
}


static int primesat_get_ext_func(RIG *rig, vfo_t vfo, token_t token, int *status)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              cfp->name);

    RETURNFUNC(RIG_OK);
}


static int primesat_set_parm(RIG *rig, setting_t parm, value_t val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int idx;
    char pstr[32];

    ENTERFUNC;

    RETURNFUNC(RIG_OK);
}


static int primesat_get_parm(RIG *rig, setting_t parm, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int idx;

    ENTERFUNC;

    RETURNFUNC(RIG_OK);
}

static int primesat_set_ext_parm(RIG *rig, token_t token, value_t val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    char lstr[64];
    const struct confparams *cfp;
    struct ext_list *epp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    RETURNFUNC(RIG_OK);
}

static int primesat_get_ext_parm(RIG *rig, token_t token, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    const struct confparams *cfp;
    struct ext_list *epp;

    ENTERFUNC;
    /* TODO: load value from priv->ext_parms */

    RETURNFUNC(RIG_OK);
}

static int primesat_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    int ret;
    freq_t freq;
    shortfreq_t ts;

    ENTERFUNC;
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              rig_strvfop(op));

    RETURNFUNC(RIG_OK);
}

static const char *primesat_get_info(RIG *rig)
{
    return "Nothing much (primesat)";
}

/*
 * primesat rig capabilities.
 */

/*
 * The following macros set bitmasks for the various funcs, levels, parms,
 * etc.  This primesat backend claims support for almost all of them.
 */
//#define primesat_FUNC  ((setting_t)-1ULL) /* All possible functions */
//#define primesat_LEVEL (((setting_t)-1ULL)&~(1ULL<<27)) /* All levels except SQLSTAT */
#define primesat_PARM  ((setting_t)-1ULL) /* All possible parms */
#define primesat_VFO_OP  (RIG_OP_CPY | RIG_OP_XCHG) /* All possible VFO OPs */
#define primesat_VFOS (RIG_VFO_MAIN| RIG_VFO_A | RIG_VFO_SUB | RIG_VFO_B)
#define primesat_MODES (RIG_MODE_LSB | RIG_MODE_USB | RIG_MODE_FM | \
                     RIG_MODE_FMN | RIG_MODE_AM | RIG_MODE_CW | \
                     RIG_MODE_CWR)

struct rig_caps primecontroller_caps =
{
    RIG_MODEL(RIG_MODEL_PRIMECONTROLLER),
    .model_name =     "PrimeSat Controller",
    .mfg_name =       "PrimeTec",
    .version =        "20230520.0",
    .copyright =      "LGPL",
    .status =         RIG_STATUS_ALPHA,
    .rig_type =       RIG_TYPE_OTHER,
    //.targetable_vfo = RIG_TARGETABLE_FREQ | RIG_TARGETABLE_MODE,
    .targetable_vfo = RIG_TARGETABLE_NONE,
    .ptt_type =       RIG_PTT_NONE,
    .dcd_type =       RIG_DCD_NONE,
    .port_type =      RIG_PORT_SERIAL,
    .serial_parity = RIG_PARITY_NONE,
    .serial_rate_min = 9600,
    .serial_rate_max = 9600,
    .serial_data_bits = 8,
    .serial_stop_bits = 1,
    .serial_handshake = RIG_HANDSHAKE_NONE,
    .write_delay =    0,
    .post_write_delay = 0,
    .timeout =    1000,
    .retry =    3,

    .has_get_func =   RIG_FUNC_NONE,
    .has_set_func =   RIG_FUNC_NONE,
    .has_get_level =  RIG_LEVEL_NONE,
    .has_set_level =  RIG_LEVEL_NONE,
    .has_get_parm =    primesat_PARM,
    .has_set_parm =    RIG_PARM_SET(primesat_PARM),
    .level_gran =      { },
    .ctcss_list =  0,
    .dcs_list =     0,
    .chan_list =   {
        RIG_CHAN_END,
    },
    .scan_ops =    RIG_SCAN_NONE,
    .vfo_ops =     primesat_VFO_OP,
    .transceive =     RIG_TRN_OFF,
    .attenuator =     { RIG_DBLST_END, },
    .preamp =          { RIG_DBLST_END, },
    .agc_level_count = 1,
    .agc_levels = {RIG_AGC_NONE},
    .rx_range_list1 =  { {
            .startf = Hz(0), .endf = MHz(9999.999999), .modes = primesat_MODES,
            .low_power = -1, .high_power = -1, primesat_VFOS, RIG_ANT_NONE,
            .label = "Primesat Controller RX range."
        },
        RIG_FRNG_END,
    },
    .tx_range_list1 =  { {
            .startf = Hz(0), .endf = MHz(9999.999999), .modes = primesat_MODES,
            .low_power = -1, .high_power = -1, primesat_VFOS, RIG_ANT_NONE,
            .label = "Primesat Controller TX range"
        },
        RIG_FRNG_END,
    },
    .tuning_steps =  { {primesat_MODES, 1}, RIG_TS_END, },
    .filters =  { RIG_FLT_END },
    .max_rit = 0,
    .max_xit = 0,
    .max_ifshift = 0,

    .spectrum_scopes = {
        {
            .id = -1,
            .name = NULL,
        },
    },
    .spectrum_modes = {
        RIG_SPECTRUM_MODE_NONE
    },
    .spectrum_spans = {
        0,
    },
    .spectrum_avg_modes = {
        {
            .id = -1,
            .name = NULL,
        },
    },
    .spectrum_attenuator = { RIG_DBLST_END, },

    .priv =  NULL,    /* priv */

    .extlevels =    primesat_ext_levels,
    .extfuncs =     primesat_ext_funcs,
    .extparms =     primesat_ext_parms,
    .cfgparams =    primesat_cfg_params,

    .rig_init =     primesat_init,
    .rig_cleanup =  primesat_cleanup,
    .rig_open =     primesat_open,
    .rig_close =    primesat_close,

    .set_conf =     primesat_set_conf,
    .get_conf =     primesat_get_conf,

    .set_freq =     primesat_set_freq,
    .get_freq =     primesat_get_freq,
    .set_mode =     primesat_set_mode,
    .get_mode =     primesat_get_mode,
    .set_vfo =      primesat_set_vfo,
    .get_vfo =      primesat_get_vfo,

    //.set_powerstat =  primesat_set_powerstat,
    //.get_powerstat =  primesat_get_powerstat,
    //.set_level =     primesat_set_level,
    //.get_level =     primesat_get_level,
    //.set_func =      primesat_set_func,
    //.get_func =      primesat_get_func,
    .set_parm =      primesat_set_parm,
    .get_parm =      primesat_get_parm,
    .set_ext_level = primesat_set_ext_level,
    .get_ext_level = primesat_get_ext_level,
    .set_ext_func =  primesat_set_ext_func,
    .get_ext_func =  primesat_get_ext_func,
    .set_ext_parm =  primesat_set_ext_parm,
    .get_ext_parm =  primesat_get_ext_parm,

    .get_info =      primesat_get_info,


    //.set_ptt =    primesat_set_ptt,
    //.get_ptt =    primesat_get_ptt,
    //.get_dcd =    primesat_get_dcd,
    //.set_rptr_shift =     primesat_set_rptr_shift,
    //.get_rptr_shift =     primesat_get_rptr_shift,
    //.set_rptr_offs =  primesat_set_rptr_offs,
    //.get_rptr_offs =  primesat_get_rptr_offs,
    //.set_ctcss_tone =     primesat_set_ctcss_tone,
    //.get_ctcss_tone =     primesat_get_ctcss_tone,
    //.set_dcs_code =   primesat_set_dcs_code,
    //.get_dcs_code =   primesat_get_dcs_code,
    //.set_ctcss_sql =  primesat_set_ctcss_sql,
    //.get_ctcss_sql =  primesat_get_ctcss_sql,
    //.set_dcs_sql =    primesat_set_dcs_sql,
    //.get_dcs_sql =    primesat_get_dcs_sql,
    .set_split_freq =     primesat_set_split_freq,
    .get_split_freq =     primesat_get_split_freq,
    .set_split_mode =     primesat_set_split_mode,
    .get_split_mode =     primesat_get_split_mode,
    .set_split_vfo =  primesat_set_split_vfo,
    .get_split_vfo =  primesat_get_split_vfo,
    //.set_rit =    primesat_set_rit,
    //.get_rit =    primesat_get_rit,
    //.set_xit =    primesat_set_xit,
    //.get_xit =    primesat_get_xit,
    .set_ts =     primesat_set_ts,
    .get_ts =     primesat_get_ts,
    //.set_ant =    primesat_set_ant,
    //.get_ant =    primesat_get_ant,
    //.set_bank =   primesat_set_bank,
    //.set_mem =    primesat_set_mem,
    //.get_mem =    primesat_get_mem,
    .vfo_op =     primesat_vfo_op,
    //.scan =       primesat_scan,
    //.send_dtmf =  primesat_send_dtmf,
    //.recv_dtmf =  primesat_recv_dtmf,
    //.send_morse =  primesat_send_morse,
    //.stop_morse =  primesat_stop_morse,
    //.send_voice_mem =  primesat_send_voice_mem,
    //.set_channel =    primesat_set_channel,
    //.get_channel =    primesat_get_channel,
    //.set_trn =    primesat_set_trn,
    //.get_trn =    primesat_get_trn,
    //.power2mW =   primesat_power2mW,
    //.mW2power =   primesat_mW2power,
    //.set_clock = primesat_set_clock,
    //.get_clock = primesat_get_clock,
    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

DECLARE_INITRIG_BACKEND(primesat)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s: _init called\n", __func__);

    rig_register(&primecontroller_caps);
    return RIG_OK;
}
