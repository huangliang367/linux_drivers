/*
 * a simple kernel module: hello
 * Copyright (C) 2014
 * Licensed under GPLv2 or later.
 * 
 */
#include <linux/init.h>
#include <linux/module.h>

static char *book_name = "Linux Device Driver";
static int book_num = 4000;

module_param(book_name, charp, S_IRUGO);
module_param(book_num, int, S_IRUGO);

static int __init hello_init(void)
{
    printk(KERN_INFO "Hello world enter!\n");
    printk(KERN_INFO "Book Name: %s, Book Num: %d\n", book_name, book_num);
    return 0;
}

static void __exit hello_exit(void)
{
    printk(KERN_INFO "Hello world exit\n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_AUTHOR("Huang Liang");
MODULE_LICENSE("GPL v2");