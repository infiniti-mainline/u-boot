// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm M31 eUSB2 High-Speed PHY, based on the Linux driver. Relies on
 * the previous bootloader to leave the supplies and repeater enabled.
 *
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <reset.h>

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

#define USB_PHY_UTMI_CTRL0		(0x3c)
#define SLEEPM				BIT(0)

#define USB_PHY_UTMI_CTRL5		(0x50)
#define POR				BIT(1)

#define USB_PHY_HS_PHY_CTRL_COMMON0	(0x54)
#define PHY_ENABLE			BIT(0)
#define SIDDQ_SEL			BIT(1)
#define SIDDQ				BIT(2)
#define FSEL				GENMASK(6, 4)
#define FSEL_38_4_MHZ_VAL		(0x6)

#define USB_PHY_HS_PHY_CTRL2		(0x64)
#define USB2_SUSPEND_N			BIT(2)
#define USB2_SUSPEND_N_SEL		BIT(3)

#define USB_PHY_CFG0			(0x94)
#define UTMI_PHY_CMN_CTRL_OVERRIDE_EN	BIT(1)

#define USB_PHY_CFG1			(0x154)
#define PLL_EN				BIT(0)

#define USB_PHY_FSEL_SEL		(0xb8)
#define FSEL_SEL			BIT(0)

#define USB_PHY_XCFGI_39_32		(0x16c)
#define HSTX_PE				GENMASK(3, 2)

#define USB_PHY_XCFGI_71_64		(0x17c)
#define HSTX_SWING			GENMASK(3, 0)

#define USB_PHY_XCFGI_31_24		(0x168)
#define HSTX_SLEW			GENMASK(2, 0)

#define USB_PHY_XCFGI_7_0		(0x15c)
#define PLL_LOCK_TIME			GENMASK(1, 0)

#define M31_EUSB_PHY_INIT_CFG(o, b, v)	\
{					\
	.off = o,			\
	.mask = b,			\
	.val = v,			\
}

struct m31_phy_tbl_entry {
	u32 off;
	u32 mask;
	u32 val;
};

static const struct m31_phy_tbl_entry m31_eusb2_setup_tbl[] = {
	M31_EUSB_PHY_INIT_CFG(USB_PHY_CFG0, UTMI_PHY_CMN_CTRL_OVERRIDE_EN, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_UTMI_CTRL5, POR, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL_COMMON0, PHY_ENABLE, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_CFG1, PLL_EN, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_FSEL_SEL, FSEL_SEL, 1),
};

static const struct m31_phy_tbl_entry m31_eusb_phy_override_tbl[] = {
	M31_EUSB_PHY_INIT_CFG(USB_PHY_XCFGI_39_32, HSTX_PE, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_XCFGI_71_64, HSTX_SWING, 7),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_XCFGI_31_24, HSTX_SLEW, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_XCFGI_7_0, PLL_LOCK_TIME, 0),
};

static const struct m31_phy_tbl_entry m31_eusb_phy_reset_tbl[] = {
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL2, USB2_SUSPEND_N_SEL, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL2, USB2_SUSPEND_N, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_UTMI_CTRL0, SLEEPM, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL_COMMON0, SIDDQ_SEL, 1),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL_COMMON0, SIDDQ, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_UTMI_CTRL5, POR, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_HS_PHY_CTRL2, USB2_SUSPEND_N_SEL, 0),
	M31_EUSB_PHY_INIT_CFG(USB_PHY_CFG0, UTMI_PHY_CMN_CTRL_OVERRIDE_EN, 0),
};

struct m31eusb2_phy_priv {
	void __iomem		*base;
	struct clk		*ref_clk;
	struct reset_ctl_bulk	resets;
	unsigned int		fsel;
};

static void m31eusb2_phy_write_mask(void __iomem *base, u32 offset,
				    u32 mask, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg &= ~mask;
	reg |= val & mask;
	writel(reg, base + offset);

	/* Ensure above write is completed */
	readl(base + offset);
}

static void m31eusb2_phy_write_sequence(struct m31eusb2_phy_priv *priv,
					const struct m31_phy_tbl_entry *tbl,
					int num)
{
	int i;

	for (i = 0; i < num; i++, tbl++)
		m31eusb2_phy_write_mask(priv->base, tbl->off, tbl->mask,
					tbl->val << __ffs(tbl->mask));
}

static int m31eusb2_phy_power_on(struct phy *phy)
{
	struct m31eusb2_phy_priv *priv = dev_get_priv(phy->dev);
	int ret;

	clk_prepare_enable(priv->ref_clk);

	/* Perform phy reset */
	reset_assert_bulk(&priv->resets);
	udelay(5);
	ret = reset_deassert_bulk(&priv->resets);
	if (ret)
		return ret;

	m31eusb2_phy_write_sequence(priv, m31_eusb2_setup_tbl,
				    ARRAY_SIZE(m31_eusb2_setup_tbl));
	m31eusb2_phy_write_mask(priv->base, USB_PHY_HS_PHY_CTRL_COMMON0,
				FSEL, FIELD_PREP(FSEL, priv->fsel));
	m31eusb2_phy_write_sequence(priv, m31_eusb_phy_override_tbl,
				    ARRAY_SIZE(m31_eusb_phy_override_tbl));
	m31eusb2_phy_write_sequence(priv, m31_eusb_phy_reset_tbl,
				    ARRAY_SIZE(m31_eusb_phy_reset_tbl));

	return 0;
}

static int m31eusb2_phy_power_off(struct phy *phy)
{
	struct m31eusb2_phy_priv *priv = dev_get_priv(phy->dev);

	reset_assert_bulk(&priv->resets);
	clk_disable_unprepare(priv->ref_clk);

	return 0;
}

static int m31eusb2_phy_probe(struct udevice *dev)
{
	struct m31eusb2_phy_priv *priv = dev_get_priv(dev);
	int ret;

	priv->base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->ref_clk = devm_clk_get(dev, "ref");
	if (IS_ERR(priv->ref_clk)) {
		ret = PTR_ERR(priv->ref_clk);
		printf("%s: failed to get ref clk %d\n", __func__, ret);
		return ret;
	}

	ret = reset_get_bulk(dev, &priv->resets);
	if (ret < 0) {
		printf("%s: failed to get resets %d\n", __func__, ret);
		return ret;
	}

	priv->fsel = FSEL_38_4_MHZ_VAL;

	return 0;
}

static struct phy_ops m31eusb2_phy_ops = {
	.power_on = m31eusb2_phy_power_on,
	.power_off = m31eusb2_phy_power_off,
};

static const struct udevice_id m31eusb2_phy_ids[] = {
	{ .compatible = "qcom,sm8750-m31-eusb2-phy" },
	{}
};

U_BOOT_DRIVER(qcom_m31_eusb2_phy) = {
	.name = "qcom-m31-eusb2-hsphy",
	.id = UCLASS_PHY,
	.of_match = m31eusb2_phy_ids,
	.ops = &m31eusb2_phy_ops,
	.probe = m31eusb2_phy_probe,
	.priv_auto = sizeof(struct m31eusb2_phy_priv),
};
