#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

class MoveTo : public MissionBlock
{
public:
	MoveTo(EveNavigator *navigator);
	~MoveTo() = default;

	void on_activation() override;
	void on_active() override;
	bool safe_alt;
private:
	float min_thresh = 1.0f;
	float max_thresh = 1.0f;
	void set_moveto_position();
};
