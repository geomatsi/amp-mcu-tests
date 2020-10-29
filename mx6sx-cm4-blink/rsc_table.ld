SECTION_PROLOGUE(".resource_table",,)
{
	. = ALIGN(4);
	KEEP(*(".resource_table"));
	. = ALIGN(4);
} GROUP_LINK_IN(RAMABLE_REGION)
