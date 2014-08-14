--[[
LuCI - Lua Configuration Interface

Copyright 2008 Steven Barth <steven@midlink.org>
Copyright 2008 Jo-Philipp Wich <xm@leipzig.freifunk.net>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

]]--
module("luci.controller.hwmon.gateworks", package.seeall)

function index()
        entry({"admin", "status", "hardware"}, call("hardware_status")).leaf = true
end

-- read current data from hwmon 
function hardware_status()

	--require "luci.tools.status"

	local has_gsp = true
	local gsp_temp
	local input_v = {}
	local input_v_name = {}

	gsp_temp = tonumber(luci.sys.exec("cat /sys/class/hwmon/hwmon0/device/in0_input"))
	if gsp_temp == -6 then
		has_gsp = false
	end

	if has_gsp then
		for i=1, 12 do                                                            
			input_v[i] = tonumber((                                     
				luci.sys.exec("cat /sys/class/hwmon/hwmon0/device/in" .. i .. "_input") or ""))
			input_v_name[i] =
				luci.sys.exec("cat /sys/class/hwmon/hwmon0/device/in" .. i .. "_label")
		end
		
		current_temp = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/hwmon0/device/temp0_input") or
			""))
		current_vin = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/hwmon0/device/in0_input") or
			""))
	else
		current_temp = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/hwmon1/device/temp1_input") or
			"")) / 100
		current_vin = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/hwmon1/device/in1_input") or
			"")) * 22.1
	end

	local rv = {
		temp       = current_temp,
		vin        = current_vin,
		input_volt = input_v,
		input_name = input_v_name
	}

	luci.http.prepare_content("application/json")
	luci.http.write_json(rv)

	return
end
