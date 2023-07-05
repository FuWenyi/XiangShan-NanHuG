sed -i '/(MET)/d' XSTop.timing.rpt

grep  "Startpoint\|Endpoint\|slack" XSTop.timing.rpt > timing.rpt
awk 'ORS=NR%3?" ":"\n"' timing.rpt > new_log.rpt

grep -n 'core_with_l2' new_log.rpt > core_with_l2.rpt
sed -i '/core_with_l2/d' new_log.rpt 

grep -n 'misc' new_log.rpt > misc.rpt
sed -i '/misc/d' new_log.rpt 

grep -n 'l3cacheOpt' new_log.rpt > l3cacheOpt.rpt
sed -i '/l3cacheOpt/d' new_log.rpt

