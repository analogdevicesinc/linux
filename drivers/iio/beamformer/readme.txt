Config for automatic power up/down at startup/reboot/powerdown:

	copy stingray_power.py to:
	/usr/share/systemd/stingray_power.py
	sudo chmod +777 /usr/share/systemd/stingray_power.py

	copy stingray.service to:
	ls -la /etc/systemd/system/stingray.service

	execute locally:
	sudo systemctl enable stingray.service --now
	Created symlink /etc/systemd/system/multi-user.target.wants/stingray.service → /etc/systemd/system/stingray.service.

Manual power up/down/partial:
	python3 /usr/share/systemd/stingray_power.py up
	python3 /usr/share/systemd/stingray_power.py down
	python3 /usr/share/systemd/stingray_power.py partial

