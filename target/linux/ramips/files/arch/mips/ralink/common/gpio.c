/*
 * Ralink SoC specific GPIO support
 *
 * Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/mach-ralink/ramips_gpio.h>

static struct ramips_gpio_data *gpio_data;

static inline struct ramips_gpio_chip *to_ramips_gpio(struct gpio_chip *chip)
{
	struct ramips_gpio_chip *rg;

	rg = container_of(chip, struct ramips_gpio_chip, chip);
	return rg;
}

static inline void ramips_gpio_wr(struct ramips_gpio_chip *rg, u8 reg, u32 val)
{
	__raw_writel(val, rg->regs_base + rg->regs[reg]);
}

static inline u32 ramips_gpio_rr(struct ramips_gpio_chip *rg, u8 reg)
{
	return __raw_readl(rg->regs_base + rg->regs[reg]);
}

static int ramips_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ramips_gpio_chip *rg = to_ramips_gpio(chip);
	unsigned long flags;
	u32 t;

	spin_lock_irqsave(&rg->lock, flags);
	t = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_DIR);
	t &= ~(1 << offset);
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_DIR, t);
	spin_unlock_irqrestore(&rg->lock, flags);

	return 0;
}

static int ramips_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	struct ramips_gpio_chip *rg = to_ramips_gpio(chip);
	unsigned long flags;
	u32 reg;
	u32 t;

	reg = (value) ? RAMIPS_GPIO_REG_SET : RAMIPS_GPIO_REG_RESET;

	spin_lock_irqsave(&rg->lock, flags);
	ramips_gpio_wr(rg, reg, 1 << offset);

	t = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_DIR);
	t |= 1 << offset;
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_DIR, t);
	spin_unlock_irqrestore(&rg->lock, flags);

	return 0;
}

static void ramips_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct ramips_gpio_chip *rg = to_ramips_gpio(chip);
	u32 reg;

	reg = (value) ? RAMIPS_GPIO_REG_SET : RAMIPS_GPIO_REG_RESET;
	ramips_gpio_wr(rg, reg, 1 << offset);
}

static int ramips_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ramips_gpio_chip *rg = to_ramips_gpio(chip);
	u32 t;

	t = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_DATA);
	return !!(t & (1 << offset));
}

int ramips_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return gpio_data->irq_base + chip->base + offset;
}

static __init void ramips_gpio_chip_add(struct ramips_gpio_chip *rg)
{
	spin_lock_init(&rg->lock);

	rg->regs_base = ioremap(rg->map_base, rg->map_size);

	rg->chip.direction_input = ramips_gpio_direction_input;
	rg->chip.direction_output = ramips_gpio_direction_output;
	rg->chip.get = ramips_gpio_get;
	rg->chip.set = ramips_gpio_set;
	rg->chip.to_irq = ramips_gpio_to_irq;

	/* set polarity to low for all lines */
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_POL, 0);
	/* set direction to input for all lines */
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_DIR, 0);
	/* clean all interrupt bits */
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_INT, 0x00FFFFFF);
	/* clean all edge bits */
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_EDGE, 0x00FFFFFF);

	gpiochip_add(&rg->chip);
}

static void ramips_gpio_irq_unmask(struct irq_data *d)
{
	int val, mask;
	struct ramips_gpio_chip *rg = d->chip_data;
	unsigned long flags;

	mask = 1 << (d->irq - gpio_data->irq_base - rg->chip.base);

	spin_lock_irqsave(&rg->lock, flags);
	if (rg->rising_edge_mask & mask){
		val = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_RENA);
		val |= mask;
		ramips_gpio_wr(rg, RAMIPS_GPIO_REG_RENA, val);
	}
	if (rg->falling_edge_mask & mask) {
		val = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_FENA);
		val |= mask;
		ramips_gpio_wr(rg, RAMIPS_GPIO_REG_FENA, val);
	}
	spin_unlock_irqrestore(&rg->lock, flags);
}

static void ramips_gpio_irq_mask(struct irq_data *d)
{
	int val, mask;
	struct ramips_gpio_chip *rg = d->chip_data;
	unsigned long flags;

	mask = 1 << (d->irq - gpio_data->irq_base - rg->chip.base);

	spin_lock_irqsave(&rg->lock, flags);
	if (rg->rising_edge_mask & mask){
		val = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_RENA);
		val &= ~mask;
		ramips_gpio_wr(rg, RAMIPS_GPIO_REG_RENA, val);
	}
	if (rg->falling_edge_mask & mask) {
		val = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_FENA);
		val &= ~mask;
		ramips_gpio_wr(rg, RAMIPS_GPIO_REG_FENA, val);
	}
	ramips_gpio_wr(rg, RAMIPS_GPIO_REG_INT, mask);
	spin_unlock_irqrestore(&rg->lock, flags);
}

static int ramips_gpio_irq_set_type(struct irq_data *d, u32 type)
{
	struct ramips_gpio_chip *rg = d->chip_data;
	unsigned int val;

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		return -EINVAL;
	}

	val = 1 << (d->irq - gpio_data->irq_base - rg->chip.base);

	if (type & IRQ_TYPE_EDGE_RISING)
		rg->rising_edge_mask |= val;
	else
		rg->rising_edge_mask &= ~val;

	if (type & IRQ_TYPE_EDGE_FALLING)
		rg->falling_edge_mask |= val;
	else
		rg->falling_edge_mask &= ~val;

	return 0;
}

static struct irq_chip ramips_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_unmask	= ramips_gpio_irq_unmask,
	.irq_mask	= ramips_gpio_irq_mask,
	.irq_mask_ack	= ramips_gpio_irq_mask,
	.irq_set_type	= ramips_gpio_irq_set_type,
};

static irqreturn_t ramips_gpio_interrupt(int irq, void *dev_id)
{
	int i, index;
	struct ramips_gpio_chip *rg;
	u32 val;

	for (i = 0; i < gpio_data->num_chips; i++) {
		rg = &gpio_data->chips[i];
		val = ramips_gpio_rr(rg, RAMIPS_GPIO_REG_INT);
		while ((index = ffs(val)) > 0) {
			do_IRQ(gpio_data->irq_base + rg->chip.base + index - 1);
			val &= ~(1 << (index-1));
		}
	}
	return IRQ_HANDLED;
}

static struct irqaction ramips_gpio_irqaction = {
	.handler	= ramips_gpio_interrupt,
	.name		= "cascade [GPIO]",
};

void __init ramips_gpio_irq_init(struct ramips_gpio_data *data)
{
	int i, j;
	int irq;
	struct ramips_gpio_chip *rg;

	gpio_data = data;

	if (data->irq_base == 0) {
		printk(KERN_WARNING "GPIO irq base not specified!!!\n");
		return;
	}

	for (i = 0; i < data->num_chips; i++) {
		rg = &data->chips[i];
		for (j = rg->chip.base; j < rg->chip.base + rg->chip.ngpio; j++) {
			irq = data->irq_base + j;
			irq_set_chip_and_handler(irq, &ramips_gpio_irq_chip, handle_level_irq);
			irq_set_chip_data(irq, rg);
			irq_set_status_flags(irq, IRQ_LEVEL);
		}
	}
	setup_irq(data->gpio_irq, &ramips_gpio_irqaction);
}

__init int ramips_gpio_init(struct ramips_gpio_data *data)
{
	int i;

	for (i = 0; i < data->num_chips; i++)
		ramips_gpio_chip_add(&data->chips[i]);

	ramips_gpio_irq_init(data);
	return 0;
}
