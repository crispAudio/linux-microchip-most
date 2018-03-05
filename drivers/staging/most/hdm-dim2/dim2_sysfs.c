/*
 * dim2_sysfs.c - MediaLB sysfs information
 *
 * Copyright (C) 2015, Microchip Technology Germany II GmbH & Co. KG
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file is licensed under GPLv2.
 */

/* Author: Andrey Shvetsov <andrey.shvetsov@k2l.de> */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/slab.h>
#include "dim2_sysfs.h"

struct bus_attr {
	struct attribute attr;
	ssize_t (*show)(struct medialb_bus *bus, char *buf);
	ssize_t (*store)(struct medialb_bus *bus, const char *buf,
			 size_t count);
};

static ssize_t state_show(struct medialb_bus *bus, char *buf)
{
	bool state = dim2_sysfs_get_state_cb();

	return sprintf(buf, "%s\n", state ? "locked" : "");
}

static struct bus_attr state_attr = __ATTR_RO(state);

static struct attribute *bus_default_attrs[] = {
	&state_attr.attr,
	NULL,
};

static const struct attribute_group bus_attr_group = {
	.attrs = bus_default_attrs,
};

static void bus_kobj_release(struct kobject *kobj)
{
	kfree(container_of(kobj, struct medialb_bus, kobj_group));
}

static ssize_t bus_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct medialb_bus *bus =
		container_of(kobj, struct medialb_bus, kobj_group);
	struct bus_attr *xattr = container_of(attr, struct bus_attr, attr);

	if (!xattr->show)
		return -EIO;

	return xattr->show(bus, buf);
}

static ssize_t bus_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
				   const char *buf, size_t count)
{
	struct medialb_bus *bus =
		container_of(kobj, struct medialb_bus, kobj_group);
	struct bus_attr *xattr = container_of(attr, struct bus_attr, attr);

	if (!xattr->store)
		return -EIO;

	return xattr->store(bus, buf, count);
}

static struct sysfs_ops const bus_kobj_sysfs_ops = {
	.show = bus_kobj_attr_show,
	.store = bus_kobj_attr_store,
};

static struct kobj_type bus_ktype = {
	.release = bus_kobj_release,
	.sysfs_ops = &bus_kobj_sysfs_ops,
};

struct dci_attr {
	struct attribute attr;
	ssize_t (*show)(struct medialb_dci *dci, char *buf);
};

static ssize_t ni_state_show(struct medialb_dci *dci, char *buf)
{
	return sprintf(buf, "%d\n", dci->ni_state);
}

static ssize_t node_position_show(struct medialb_dci *dci, char *buf)
{
	return sprintf(buf, "%d\n", dci->node_position);
}

static ssize_t node_address_show(struct medialb_dci *dci, char *buf)
{
	return sprintf(buf, "0x%04x\n", dci->node_address);
}

static struct dci_attr dci_attrs[] = {
	__ATTR_RO(ni_state),
	__ATTR_RO(node_position),
	__ATTR_RO(node_address),
};

static struct attribute *dci_default_attrs[] = {
	&dci_attrs[0].attr,
	&dci_attrs[1].attr,
	&dci_attrs[2].attr,
	NULL,
};

static const struct attribute_group dci_attr_group = {
	.attrs = dci_default_attrs,
};

static void dci_kobj_release(struct kobject *kobj)
{
	kfree(container_of(kobj, struct medialb_dci, kobj_group));
}

static ssize_t dci_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct medialb_dci *dci =
		container_of(kobj, struct medialb_dci, kobj_group);
	struct dci_attr *xattr = container_of(attr, struct dci_attr, attr);
	ssize_t ret;

	if (!xattr->show)
		return -EIO;

	mutex_lock(&dci->mt);
	ret = xattr->show(dci, buf);
	mutex_unlock(&dci->mt);
	return ret;
}

static struct sysfs_ops const dci_kobj_sysfs_ops = {
	.show = dci_kobj_attr_show,
};

static struct kobj_type dci_ktype = {
	.release = dci_kobj_release,
	.sysfs_ops = &dci_kobj_sysfs_ops,
};

int dim2_sysfs_probe(struct medialb_bus **busp, struct medialb_dci **dcip,
		     struct kobject *parent_kobj)
{
	int err;
	struct medialb_bus *bus;
	struct medialb_dci *dci;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	err = kobject_init_and_add(&bus->kobj_group,
				   &bus_ktype, parent_kobj, "bus");
	if (err) {
		pr_err("kobject_add() failed: %d\n", err);
		goto err_kobject_add;
	}

	err = sysfs_create_group(&bus->kobj_group, &bus_attr_group);
	if (err) {
		pr_err("sysfs_create_group() failed: %d\n", err);
		goto err_create_group;
	}

	dci = kzalloc(sizeof(*dci), GFP_KERNEL);
	if (!dci) {
		err = -ENOMEM;
		goto err_kzalloc;
	}

	mutex_init(&dci->mt);

	err = kobject_init_and_add(&dci->kobj_group,
				   &dci_ktype, parent_kobj, "dci");
	if (err) {
		pr_err("kobject_add() failed: %d\n", err);
		goto err_kobject_add_dci;
	}

	err = sysfs_create_group(&dci->kobj_group, &dci_attr_group);
	if (err) {
		pr_err("sysfs_create_group() failed: %d\n", err);
		goto err_create_dci_group;
	}

	*busp = bus;
	*dcip = dci;
	return 0;

err_create_dci_group:
	kobject_put(&dci->kobj_group);

err_kobject_add_dci:
	kfree(dci);

err_kzalloc:
err_create_group:
	kobject_put(&bus->kobj_group);

err_kobject_add:
	kfree(bus);
	return err;
}

void dim2_sysfs_destroy(struct medialb_bus *bus, struct medialb_dci *dci)
{
	kobject_put(&dci->kobj_group);
	kobject_put(&bus->kobj_group);
}
