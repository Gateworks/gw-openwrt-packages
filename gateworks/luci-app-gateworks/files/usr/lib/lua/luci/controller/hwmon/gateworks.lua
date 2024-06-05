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
	local gsc_dev
	local input_v = {}
	local input_v_name = {}
	for f in string.gmatch(luci.sys.exec("ls /sys/class/hwmon/"), "[^%s]+") do
		if luci.sys.exec("cat /sys/class/hwmon/" .. f .. "/name") == "gsc_hwmon\n" then
			gsc_dev = f
		end
	end
	if gsc_dev ~= nil then
		for i=0, 12 do
			input_v[i] = tonumber((
				luci.sys.exec("cat /sys/class/hwmon/" .. gsc_dev .. "/in" .. i .. "_input") or ""))
			input_v_name[i] =
				luci.sys.exec("cat /sys/class/hwmon/" .. gsc_dev .. "/in" .. i .. "_label")
		end

		current_temp = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/" .. gsc_dev .. "/temp1_input") or
			""))
		current_vin = tonumber((
			luci.sys.exec("cat /sys/class/hwmon/" .. gsc_dev .. "/in0_input") or
			""))
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
