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
    ptt_t ptt;
    powerstat_t powerstat;
    int bank;
    value_t parms[RIG_SETTING_MAX];
    int ant_option[4]; /* simulate 4 antennas */
    int trn; /* transceive */

    channel_t *curr;    /* points to vfo_a, vfo_b or mem[] */

    // we're trying to emulate all sorts of vfo possibilities so this looks redundant
    channel_t vfo_a;
    channel_t vfo_b;
    channel_t vfo_c;
    channel_t vfo_maina;
    channel_t vfo_mainb;
    channel_t vfo_suba;
    channel_t vfo_subb;
    channel_t mem[NB_CHAN];

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

static void init_chan(RIG *rig, vfo_t vfo, channel_t *chan)
{
    chan->channel_num = 0;
    chan->vfo = vfo;
    strcpy(chan->channel_desc, rig_strvfo(vfo));

    switch (vfo)
    {
    case RIG_VFO_A:
    case RIG_VFO_MAIN_A:
        chan->freq = MHz(145);
        break;

    case RIG_VFO_B:
    case RIG_VFO_MAIN_B:
        chan->freq = MHz(146);
        break;

    case RIG_VFO_SUB_A:
        chan->freq = MHz(147);
        break;

    case RIG_VFO_SUB_B:
        chan->freq = MHz(148);
        break;

    default:
        rig_debug(RIG_DEBUG_ERR, "%s(%d) unknown vfo=%s\n", __FILE__, __LINE__,
                  rig_strvfo(vfo));
    }

    chan->mode = RIG_MODE_FM;
    chan->width = rig_passband_normal(rig, RIG_MODE_FM);
    chan->tx_freq = chan->freq;
    chan->tx_mode = chan->mode;
    chan->tx_width = chan->width;
    chan->split = RIG_SPLIT_OFF;
    chan->tx_vfo = vfo;

    chan->rptr_shift = RIG_RPT_SHIFT_NONE;
    chan->rptr_offs = 0;
    chan->ctcss_tone = 0;
    chan->dcs_code = 0;
    chan->ctcss_sql = 0;
    chan->dcs_sql = 0;
    chan->rit = 0;
    chan->xit = 0;
    chan->tuning_step = 0;
    chan->ant = 0;

    chan->funcs = (setting_t)0;
    memset(chan->levels, 0, RIG_SETTING_MAX * sizeof(value_t));
}

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
    rig->state.rigport.type.rig = RIG_PORT_NONE;

    priv->split = RIG_SPLIT_OFF;
    priv->ptt = RIG_PTT_OFF;
    priv->powerstat = RIG_POWER_ON;
    rig->state.powerstat = priv->powerstat;
    priv->bank = 0;
    memset(priv->parms, 0, RIG_SETTING_MAX * sizeof(value_t));

    memset(priv->mem, 0, sizeof(priv->mem));

    for (i = 0; i < NB_CHAN; i++)
    {
        priv->mem[i].channel_num = i;
        priv->mem[i].vfo = RIG_VFO_MEM;

        priv->mem[i].ext_levels = alloc_init_ext(primesat_ext_levels);

        if (!priv->mem[i].ext_levels)
        {
            RETURNFUNC(-RIG_ENOMEM);
        }
    }

    priv->vfo_a.ext_levels = alloc_init_ext(primesat_ext_levels);

    if (!priv->vfo_a.ext_levels)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    priv->vfo_b.ext_levels = alloc_init_ext(primesat_ext_levels);

    if (!priv->vfo_b.ext_levels)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    priv->ext_funcs = alloc_init_ext(primesat_ext_funcs);

    if (!priv->ext_funcs)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    priv->ext_parms = alloc_init_ext(primesat_ext_parms);

    if (!priv->ext_parms)
    {
        RETURNFUNC(-RIG_ENOMEM);
    }

    init_chan(rig, RIG_VFO_A, &priv->vfo_a);
    init_chan(rig, RIG_VFO_B, &priv->vfo_b);
    init_chan(rig, RIG_VFO_MAIN_A, &priv->vfo_maina);
    init_chan(rig, RIG_VFO_MAIN_B, &priv->vfo_mainb);
    init_chan(rig, RIG_VFO_SUB_A, &priv->vfo_suba);
    init_chan(rig, RIG_VFO_SUB_B, &priv->vfo_subb);
    priv->curr = &priv->vfo_a;

    if (rig->caps->rig_model == RIG_MODEL_primesat_NOVFO)
    {
        priv->curr_vfo = priv->last_vfo = RIG_VFO_CURR;
    }
    else
    {
        priv->curr_vfo = priv->last_vfo = RIG_VFO_A;
    }

    priv->magic_conf = strdup("DX");

    RETURNFUNC(RIG_OK);
}

static int primesat_cleanup(RIG *rig)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int i;

    ENTERFUNC;

    for (i = 0; i < NB_CHAN; i++)
    {
        free(priv->mem[i].ext_levels);
    }

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

    if (rig->caps->rig_model == RIG_MODEL_primesat_NOVFO)
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

    switch (token)
    {
    case TOK_CFG_MAGICCONF:
        if (val)
        {
            free(priv->magic_conf);
            priv->magic_conf = strdup(val);
        }

        break;

    case TOK_CFG_STATIC_DATA:
        priv->static_data = atoi(val) ? 1 : 0;
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    RETURNFUNC(RIG_OK);
}

static int primesat_get_conf(RIG *rig, token_t token, char *val)
{
    struct primesat_priv_data *priv;

    ENTERFUNC;
    priv = (struct primesat_priv_data *)rig->state.priv;

    switch (token)
    {
    case TOK_CFG_MAGICCONF:
        strcpy(val, priv->magic_conf);
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    RETURNFUNC(RIG_OK);
}

static int primesat_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    char fstr[20];

    ENTERFUNC;

    if (rig == NULL)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: rig is NULL!!!\n", __func__);
        return -RIG_EINVAL;
    }

    if (vfo == RIG_VFO_CURR) { vfo = priv->curr_vfo; }

    if (vfo == RIG_VFO_CURR || vfo == RIG_VFO_TX) { vfo = vfo_fixup(rig, vfo, rig->state.cache.split); }

// if needed for testing enable this to emulate a rig with 100hz resolution
#if 0
    // we emulate a rig with 100Hz set freq interval limits -- truncation
    freq = freq - fmod(freq, 100);
#endif
    usleep(CMDSLEEP);
    sprintf_freq(fstr, sizeof(fstr), freq);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s\n", __func__,
              rig_strvfo(vfo), fstr);

    switch (vfo)
    {
    case RIG_VFO_MAIN:
    case RIG_VFO_A: priv->vfo_a.freq = freq; break;

    case RIG_VFO_MAIN_A: priv->vfo_maina.freq = freq; break;

    case RIG_VFO_MAIN_B: priv->vfo_mainb.freq = freq; break;

    case RIG_VFO_SUB:
    case RIG_VFO_B: priv->vfo_b.freq = freq; break;

    case RIG_VFO_SUB_A: priv->vfo_suba.freq = freq; break;

    case RIG_VFO_SUB_B: priv->vfo_subb.freq = freq; break;

    case RIG_VFO_C: priv->vfo_c.freq = freq; break;
    }

    if (priv && !priv->split)
    {
        priv->curr->tx_freq = freq;
    }

    rig_debug(RIG_DEBUG_TRACE, "%s: curr->freq=%.0f, curr->tx_freq=%.0f\n",
              __func__,
              priv->curr->freq, priv->curr->tx_freq);
    RETURNFUNC(RIG_OK);
}


static int primesat_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;

    if (vfo == RIG_VFO_CURR && rig->caps->rig_model != RIG_MODEL_primesat_NOVFO) { vfo = priv->curr_vfo; }

    if ((vfo == RIG_VFO_SUB && rig->state.uplink == 1)
            || (vfo == RIG_VFO_MAIN && rig->state.uplink == 2))
    {
        rig_debug(RIG_DEBUG_TRACE, "%s: uplink=%d, ignoring get_freq\n", __func__,
                  rig->state.uplink);
        RETURNFUNC(RIG_OK);
    }

    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    switch (vfo)
    {
    case RIG_VFO_MAIN:
    case RIG_VFO_A:  *freq = priv->vfo_a.freq; break;

    case RIG_VFO_MAIN_A: *freq = priv->vfo_maina.freq; break;

    case RIG_VFO_MAIN_B: *freq = priv->vfo_mainb.freq; break;

    case RIG_VFO_SUB:
    case RIG_VFO_B:  *freq = priv->vfo_b.freq; break;

    case RIG_VFO_SUB_A:  *freq = priv->vfo_suba.freq; break;

    case RIG_VFO_SUB_B:  *freq = priv->vfo_subb.freq; break;

    case RIG_VFO_C:  *freq = priv->vfo_c.freq; break;

    default: RETURNFUNC(-RIG_EINVAL);
    }

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
    sprintf_freq(buf, sizeof(buf), width);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s %s\n", __func__,
              rig_strvfo(vfo), rig_strrmode(mode), buf);

    vfo = vfo_fixup(rig, vfo, rig->state.cache.split);

    if (width == RIG_PASSBAND_NOCHANGE)
    {
        switch (vfo)
        {
        case RIG_VFO_MAIN:
        case RIG_VFO_A: width = priv->vfo_a.width; break;

        case RIG_VFO_SUB:
        case RIG_VFO_B: width = priv->vfo_b.width; break;

        case RIG_VFO_C: width = priv->vfo_c.width; break;
        }
    }

    switch (vfo)
    {
    case RIG_VFO_MAIN:
    case RIG_VFO_A: priv->vfo_a.mode = mode; priv->vfo_a.width = width; break;

    case RIG_VFO_SUB:
    case RIG_VFO_B: priv->vfo_b.mode = mode; priv->vfo_b.width = width; break;

    case RIG_VFO_C: priv->vfo_c.mode = mode; priv->vfo_c.width = width; break;

    default:
        rig_debug(RIG_DEBUG_ERR, "%s: unknown VFO=%s\n", __func__, rig_strvfo(vfo));
        RETURNFUNC(-RIG_EINVAL);
    }

    vfo = vfo_fixup(rig, vfo, rig->state.cache.split);

    if (RIG_PASSBAND_NOCHANGE == width) { RETURNFUNC(RIG_OK); }

    if (width == RIG_PASSBAND_NORMAL)
    {
        width = curr->width = rig_passband_normal(rig, mode);
    }

    switch (vfo)
    {
    case RIG_VFO_A: priv->vfo_a.width = width; break;

    case RIG_VFO_B: priv->vfo_b.width = width; break;

    case RIG_VFO_C: priv->vfo_c.width = width; break;
    }

    RETURNFUNC(RIG_OK);
}


static int primesat_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    if (vfo == RIG_VFO_CURR) { vfo = rig->state.current_vfo; }

    switch (vfo)
    {
    case RIG_VFO_MAIN:
    case RIG_VFO_A: *mode = priv->vfo_a.mode; *width = priv->vfo_a.width; break;

    case RIG_VFO_SUB:
    case RIG_VFO_B: *mode = priv->vfo_b.mode; *width = priv->vfo_b.width; break;

    case RIG_VFO_C: *mode = priv->vfo_c.mode; *width = priv->vfo_c.width; break;
    }

    RETURNFUNC(RIG_OK);
}


static int primesat_set_vfo(RIG *rig, vfo_t vfo)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    usleep(CMDSLEEP);
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__, rig_strvfo(vfo));

    if (vfo == RIG_VFO_CURR) { vfo = rig->state.current_vfo; }

    priv->last_vfo = priv->curr_vfo;
    priv->curr_vfo = vfo;

    switch (vfo)
    {
    case RIG_VFO_VFO: /* FIXME */

    case RIG_VFO_RX:
    case RIG_VFO_MAIN: priv->curr = &priv->vfo_a; break;

    case RIG_VFO_MAIN_A: priv->curr = &priv->vfo_maina; break;

    case RIG_VFO_MAIN_B: priv->curr = &priv->vfo_mainb; break;

    case RIG_VFO_A: priv->curr = &priv->vfo_a; break;

    case RIG_VFO_SUB: priv->curr = &priv->vfo_b; break;

    case RIG_VFO_SUB_A: priv->curr = &priv->vfo_suba; break;

    case RIG_VFO_SUB_B: priv->curr = &priv->vfo_subb; break;

    case RIG_VFO_B: priv->curr = &priv->vfo_b; break;

    case RIG_VFO_C: priv->curr = &priv->vfo_c; break;

    case RIG_VFO_MEM:
        if (curr->channel_num >= 0 && curr->channel_num < NB_CHAN)
        {
            priv->curr = &priv->mem[curr->channel_num];
            break;
        }

    case RIG_VFO_TX:
        if (priv->tx_vfo == RIG_VFO_A) { priv->curr = &priv->vfo_a; }
        else if (priv->tx_vfo == RIG_VFO_B) { priv->curr = &priv->vfo_b; }
        else if (priv->tx_vfo == RIG_VFO_MEM) { priv->curr = &priv->mem[curr->channel_num]; }
        else { priv->curr = &priv->vfo_a; }

        break;

    default:
        rig_debug(RIG_DEBUG_VERBOSE, "%s unknown vfo: %s\n", __func__,
                  rig_strvfo(vfo));
        RETURNFUNC(-RIG_EINVAL);
    }

    rig->state.current_vfo = vfo;

    RETURNFUNC(RIG_OK);
}


static int primesat_get_vfo(RIG *rig, vfo_t *vfo)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_dcd(RIG *rig, vfo_t vfo, dcd_t *dcd)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t rptr_shift)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t *rptr_shift)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t rptr_offs)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t *rptr_offs)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_ctcss_tone(RIG *rig, vfo_t vfo, tone_t tone)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_ctcss_tone(RIG *rig, vfo_t vfo, tone_t *tone)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_dcs_code(RIG *rig, vfo_t vfo, tone_t code)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_dcs_code(RIG *rig, vfo_t vfo, tone_t *code)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_ctcss_sql(RIG *rig, vfo_t vfo, tone_t tone)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_ctcss_sql(RIG *rig, vfo_t vfo, tone_t *tone)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_dcs_sql(RIG *rig, vfo_t vfo, unsigned int code)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_dcs_sql(RIG *rig, vfo_t vfo, unsigned int *code)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_split_freq(RIG *rig, vfo_t vfo, freq_t tx_freq)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_split_freq(RIG *rig, vfo_t vfo, freq_t *tx_freq)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_set_split_mode(RIG *rig, vfo_t vfo, rmode_t tx_mode,
                                pbwidth_t tx_width)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_get_split_mode(RIG *rig, vfo_t vfo, rmode_t *tx_mode,
                                pbwidth_t *tx_width)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_set_split_vfo(RIG *rig, vfo_t vfo, split_t split, vfo_t tx_vfo)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_split_vfo(RIG *rig, vfo_t vfo, split_t *split,
                               vfo_t *tx_vfo)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_set_rit(RIG *rig, vfo_t vfo, shortfreq_t rit)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_rit(RIG *rig, vfo_t vfo, shortfreq_t *rit)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_xit(RIG *rig, vfo_t vfo, shortfreq_t xit)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_xit(RIG *rig, vfo_t vfo, shortfreq_t *xit)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
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


static int primesat_set_func(RIG *rig, vfo_t vfo, setting_t func, int status)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %d\n", __func__,
              rig_strfunc(func), status);

    if (status)
    {
        curr->funcs |=  func;
    }
    else
    {
        curr->funcs &= ~func;
    }

    RETURNFUNC(RIG_OK);
}


static int primesat_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;

    ENTERFUNC;
    *status = (curr->funcs & func) ? 1 : 0;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              rig_strfunc(func));

    RETURNFUNC(RIG_OK);
}


static int primesat_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_set_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    char lstr[64];
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EL_MAGICLEVEL:
    case TOK_EL_MAGICFUNC:
    case TOK_EL_MAGICOP:
    case TOK_EL_MAGICCOMBO:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (cfp->type)
    {
    case RIG_CONF_STRING:
        strcpy(lstr, val.s);
        break;

    case RIG_CONF_COMBO:
        SNPRINTF(lstr, sizeof(lstr), "%d", val.i);
        break;

    case RIG_CONF_NUMERIC:
        SNPRINTF(lstr, sizeof(lstr), "%f", val.f);
        break;

    case RIG_CONF_CHECKBUTTON:
        SNPRINTF(lstr, sizeof(lstr), "%s", val.i ? "ON" : "OFF");
        break;

    case RIG_CONF_BUTTON:
        lstr[0] = '\0';
        break;

    default:
        RETURNFUNC(-RIG_EINTERNAL);
    }

    elp = find_ext(curr->ext_levels, token);

    if (!elp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* store value */
    elp->val = val;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s\n", __func__,
              cfp->name, lstr);

    RETURNFUNC(RIG_OK);
}

static int primesat_get_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    channel_t *curr = priv->curr;
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EL_MAGICLEVEL:
    case TOK_EL_MAGICFUNC:
    case TOK_EL_MAGICOP:
    case TOK_EL_MAGICCOMBO:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    elp = find_ext(curr->ext_levels, token);

    if (!elp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* load value */
    *val = elp->val;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              cfp->name);

    RETURNFUNC(RIG_OK);
}


static int primesat_set_ext_func(RIG *rig, vfo_t vfo, token_t token, int status)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    const struct confparams *cfp;
    struct ext_list *elp;

    ENTERFUNC;
    cfp = rig_ext_lookup_tok(rig, token);

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EL_MAGICEXTFUNC:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (cfp->type)
    {
    case RIG_CONF_CHECKBUTTON:
        break;

    case RIG_CONF_BUTTON:
        break;

    default:
        RETURNFUNC(-RIG_EINTERNAL);
    }

    elp = find_ext(priv->ext_funcs, token);

    if (!elp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* store value */
    elp->val.i = status;

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

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EL_MAGICEXTFUNC:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    elp = find_ext(priv->ext_funcs, token);

    if (!elp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* load value */
    *status = elp->val.i;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              cfp->name);

    RETURNFUNC(RIG_OK);
}


static int primesat_set_powerstat(RIG *rig, powerstat_t status)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_powerstat(RIG *rig, powerstat_t *status)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_parm(RIG *rig, setting_t parm, value_t val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int idx;
    char pstr[32];

    ENTERFUNC;
    idx = rig_setting2idx(parm);

    if (idx >= RIG_SETTING_MAX)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    if (RIG_PARM_IS_FLOAT(parm))
    {
        SNPRINTF(pstr, sizeof(pstr), "%f", val.f);
    }
    else
    {
        SNPRINTF(pstr, sizeof(pstr), "%d", val.i);
    }

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s\n", __func__,
              rig_strparm(parm), pstr);
    priv->parms[idx] = val;

    RETURNFUNC(RIG_OK);
}


static int primesat_get_parm(RIG *rig, setting_t parm, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    int idx;

    ENTERFUNC;
    idx = rig_setting2idx(parm);

    if (idx >= RIG_SETTING_MAX)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    *val = priv->parms[idx];
    rig_debug(RIG_DEBUG_VERBOSE, "%s called %s\n", __func__,
              rig_strparm(parm));

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

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EP_MAGICPARM:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (cfp->type)
    {
    case RIG_CONF_STRING:
        strcpy(lstr, val.s);
        break;

    case RIG_CONF_COMBO:
        SNPRINTF(lstr, sizeof(lstr), "%d", val.i);
        break;

    case RIG_CONF_NUMERIC:
        SNPRINTF(lstr, sizeof(lstr), "%f", val.f);
        break;

    case RIG_CONF_CHECKBUTTON:
        SNPRINTF(lstr, sizeof(lstr), "%s", val.i ? "ON" : "OFF");
        break;

    case RIG_CONF_BUTTON:
        lstr[0] = '\0';
        break;

    default:
        RETURNFUNC(-RIG_EINTERNAL);
    }

    epp = find_ext(priv->ext_parms, token);

    if (!epp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* store value */
    epp->val = val;


    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s %s\n", __func__,
              cfp->name, lstr);

    RETURNFUNC(RIG_OK);
}

static int primesat_get_ext_parm(RIG *rig, token_t token, value_t *val)
{
    struct primesat_priv_data *priv = (struct primesat_priv_data *)rig->state.priv;
    const struct confparams *cfp;
    struct ext_list *epp;

    ENTERFUNC;
    /* TODO: load value from priv->ext_parms */

    cfp = rig_ext_lookup_tok(rig, token);

    if (!cfp)
    {
        RETURNFUNC(-RIG_EINVAL);
    }

    switch (token)
    {
    case TOK_EP_MAGICPARM:
        break;

    default:
        RETURNFUNC(-RIG_EINVAL);
    }

    epp = find_ext(priv->ext_parms, token);

    if (!epp)
    {
        RETURNFUNC(-RIG_EINTERNAL);
    }

    /* load value */
    *val = epp->val;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called: %s\n", __func__,
              cfp->name);

    RETURNFUNC(RIG_OK);
}



static int primesat_set_ant(RIG *rig, vfo_t vfo, ant_t ant, value_t option)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_ant(RIG *rig, vfo_t vfo, ant_t ant, value_t *option,
                         ant_t *ant_curr, ant_t *ant_tx, ant_t *ant_rx)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_bank(RIG *rig, vfo_t vfo, int bank)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_mem(RIG *rig, vfo_t vfo, int ch)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_mem(RIG *rig, vfo_t vfo, int *ch)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_scan(RIG *rig, vfo_t vfo, scan_t scan, int ch)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static void chan_vfo(channel_t *chan, vfo_t vfo)
{
    RETURNFUNC(-RIG_ENAVAIL);
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

    switch (op)
    {
    case RIG_OP_FROM_VFO: /* VFO->MEM */
        if (priv->curr_vfo == RIG_VFO_MEM)
        {
            int ch = curr->channel_num;
            copy_chan(curr, priv->last_vfo == RIG_VFO_A ?
                      &priv->vfo_a : &priv->vfo_b);
            curr->channel_num = ch;
            curr->channel_desc[0] = '\0';
            curr->vfo = RIG_VFO_MEM;
        }
        else
        {
            channel_t *mem_chan = &priv->mem[curr->channel_num];
            copy_chan(mem_chan, curr);
            mem_chan->channel_num = curr->channel_num;
            mem_chan->channel_desc[0] = '\0';
            mem_chan->vfo = RIG_VFO_MEM;
        }

        break;

    case RIG_OP_TO_VFO:         /* MEM->VFO */
        if (priv->curr_vfo == RIG_VFO_MEM)
        {
            channel_t *vfo_chan = (priv->last_vfo == RIG_VFO_A) ?
                                  &priv->vfo_a : &priv->vfo_b;
            copy_chan(vfo_chan, curr);
            chan_vfo(vfo_chan, priv->last_vfo);
        }
        else
        {
            copy_chan(&priv->mem[curr->channel_num], curr);
            chan_vfo(curr, priv->curr_vfo);
        }

        break;

    case RIG_OP_CPY:   /* VFO A = VFO B   or   VFO B = VFO A */
        if (priv->curr_vfo == RIG_VFO_A)
        {
            copy_chan(&priv->vfo_b, &priv->vfo_a);
            chan_vfo(&priv->vfo_b, RIG_VFO_B);
            break;
        }
        else if (priv->curr_vfo == RIG_VFO_B)
        {
            copy_chan(&priv->vfo_a, &priv->vfo_b);
            chan_vfo(&priv->vfo_a, RIG_VFO_A);
            break;
        }

        rig_debug(RIG_DEBUG_VERBOSE, "%s beep!\n", __func__);
        break;

    case RIG_OP_XCHG: /* Exchange VFO A/B */
    {
        channel_t chan;
        chan.ext_levels = alloc_init_ext(primesat_ext_levels);

        if (!chan.ext_levels)
        {
            RETURNFUNC(-RIG_ENOMEM);
        }

        copy_chan(&chan, &priv->vfo_b);
        copy_chan(&priv->vfo_b, &priv->vfo_a);
        copy_chan(&priv->vfo_a, &chan);
        chan_vfo(&priv->vfo_a, RIG_VFO_A);
        chan_vfo(&priv->vfo_b, RIG_VFO_B);
        free(chan.ext_levels);
        break;
    }

    case RIG_OP_MCL:  /* Memory clear */
        if (priv->curr_vfo == RIG_VFO_MEM)
        {
            struct ext_list *saved_ext_levels = curr->ext_levels;
            int saved_ch = curr->channel_num;
            int i;

            for (i = 0; !RIG_IS_EXT_END(curr->ext_levels[i]); i++)
            {
                curr->ext_levels[i].val.i = 0;
            }

            memset(curr, 0, sizeof(channel_t));
            curr->ext_levels = saved_ext_levels;
            curr->channel_num = saved_ch;
            curr->vfo = RIG_VFO_MEM;
        }
        else
        {
            struct ext_list *saved_ext_levels = curr->ext_levels;
            channel_t *mem_chan = &priv->mem[curr->channel_num];
            int i;

            for (i = 0; !RIG_IS_EXT_END(mem_chan->ext_levels[i]); i++)
            {
                mem_chan->ext_levels[i].val.i = 0;
            }

            memset(mem_chan, 0, sizeof(channel_t));
            mem_chan->ext_levels = saved_ext_levels;
            mem_chan->channel_num = curr->channel_num;
            mem_chan->vfo = RIG_VFO_MEM;
        }

        break;

    case RIG_OP_TOGGLE:
        if (priv->curr_vfo == RIG_VFO_A)
        {
            RETURNFUNC(primesat_set_vfo(rig, RIG_VFO_B));
        }
        else if (priv->curr_vfo == RIG_VFO_B)
        {
            RETURNFUNC(primesat_set_vfo(rig, RIG_VFO_A));
        }
        else
        {
            RETURNFUNC(-RIG_EVFO);
        }

    case RIG_OP_RIGHT:
    case RIG_OP_LEFT:
    case RIG_OP_TUNE:
        /* NOP */
        break;

    case RIG_OP_BAND_UP:
    case RIG_OP_BAND_DOWN:
        RETURNFUNC(-RIG_ENIMPL);

    case RIG_OP_UP:
        ret = primesat_get_freq(rig, vfo, &freq);

        if (!ret) { break; }

        ret = primesat_get_ts(rig, vfo, &ts);

        if (!ret) { break; }

        primesat_set_freq(rig, vfo, freq + ts);  /* up */
        break;

    case RIG_OP_DOWN:
        ret = primesat_get_freq(rig, vfo, &freq);

        if (!ret) { break; }

        ret = primesat_get_ts(rig, vfo, &ts);

        if (!ret) { break; }

        primesat_set_freq(rig, vfo, freq - ts);  /* down */
        break;

    default:
        break;
    }

    RETURNFUNC(RIG_OK);
}

static int primesat_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_get_channel(RIG *rig, vfo_t vfo, channel_t *chan,
                             int read_only)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_set_trn(RIG *rig, int trn)
{

    RETURNFUNC2(-RIG_ENAVAIL);
}


static int primesat_get_trn(RIG *rig, int *trn)
{
    RETURNFUNC2(-RIG_ENAVAIL);
}

static const char *primesat_get_info(RIG *rig)
{
    return "Nothing much (primesat)";
}


static int primesat_send_dtmf(RIG *rig, vfo_t vfo, const char *digits)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_recv_dtmf(RIG *rig, vfo_t vfo, char *digits, int *length)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_send_morse(RIG *rig, vfo_t vfo, const char *msg)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_stop_morse(RIG *rig, vfo_t vfo)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_send_voice_mem(RIG *rig, vfo_t vfo, int ch)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

static int primesat_power2mW(RIG *rig, unsigned int *mwpower, float power,
                          freq_t freq, rmode_t mode)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}


static int primesat_mW2power(RIG *rig, float *power, unsigned int mwpower,
                          freq_t freq, rmode_t mode)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

//static int m_year, m_month, m_day, m_hour, m_min, m_sec, m_utc_offset;
//static double m_msec;

int primesat_set_clock(RIG *rig, int year, int month, int day, int hour, int min,
                    int sec, double msec, int utc_offset)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
}

int primesat_get_clock(RIG *rig, int *year, int *month, int *day, int *hour,
                    int *min, int *sec, double *msec, int *utc_offset)
{
    ENTERFUNC;
    RETURNFUNC(-RIG_ENAVAIL);
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
#define primesat_VFOS (RIG_VFO_MAIN|RIG_VFO_SUB)
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

    .set_powerstat =  primesat_set_powerstat,
    .get_powerstat =  primesat_get_powerstat,
    .set_level =     primesat_set_level,
    .get_level =     primesat_get_level,
    .set_func =      primesat_set_func,
    .get_func =      primesat_get_func,
    .set_parm =      primesat_set_parm,
    .get_parm =      primesat_get_parm,
    .set_ext_level = primesat_set_ext_level,
    .get_ext_level = primesat_get_ext_level,
    .set_ext_func =  primesat_set_ext_func,
    .get_ext_func =  primesat_get_ext_func,
    .set_ext_parm =  primesat_set_ext_parm,
    .get_ext_parm =  primesat_get_ext_parm,

    .get_info =      primesat_get_info,


    .set_ptt =    primesat_set_ptt,
    .get_ptt =    primesat_get_ptt,
    .get_dcd =    primesat_get_dcd,
    .set_rptr_shift =     primesat_set_rptr_shift,
    .get_rptr_shift =     primesat_get_rptr_shift,
    .set_rptr_offs =  primesat_set_rptr_offs,
    .get_rptr_offs =  primesat_get_rptr_offs,
    .set_ctcss_tone =     primesat_set_ctcss_tone,
    .get_ctcss_tone =     primesat_get_ctcss_tone,
    .set_dcs_code =   primesat_set_dcs_code,
    .get_dcs_code =   primesat_get_dcs_code,
    .set_ctcss_sql =  primesat_set_ctcss_sql,
    .get_ctcss_sql =  primesat_get_ctcss_sql,
    .set_dcs_sql =    primesat_set_dcs_sql,
    .get_dcs_sql =    primesat_get_dcs_sql,
    .set_split_freq =     primesat_set_split_freq,
    .get_split_freq =     primesat_get_split_freq,
    .set_split_mode =     primesat_set_split_mode,
    .get_split_mode =     primesat_get_split_mode,
    .set_split_vfo =  primesat_set_split_vfo,
    .get_split_vfo =  primesat_get_split_vfo,
    .set_rit =    primesat_set_rit,
    .get_rit =    primesat_get_rit,
    .set_xit =    primesat_set_xit,
    .get_xit =    primesat_get_xit,
    .set_ts =     primesat_set_ts,
    .get_ts =     primesat_get_ts,
    .set_ant =    primesat_set_ant,
    .get_ant =    primesat_get_ant,
    .set_bank =   primesat_set_bank,
    .set_mem =    primesat_set_mem,
    .get_mem =    primesat_get_mem,
    .vfo_op =     primesat_vfo_op,
    .scan =       primesat_scan,
    .send_dtmf =  primesat_send_dtmf,
    .recv_dtmf =  primesat_recv_dtmf,
    .send_morse =  primesat_send_morse,
    .stop_morse =  primesat_stop_morse,
    .send_voice_mem =  primesat_send_voice_mem,
    .set_channel =    primesat_set_channel,
    .get_channel =    primesat_get_channel,
    .set_trn =    primesat_set_trn,
    .get_trn =    primesat_get_trn,
    .power2mW =   primesat_power2mW,
    .mW2power =   primesat_mW2power,
    .set_clock = primesat_set_clock,
    .get_clock = primesat_get_clock,
    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

DECLARE_INITRIG_BACKEND(primesat)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s: _init called\n", __func__);

    rig_register(&primecontroller_caps);
    return RIG_OK;
}
