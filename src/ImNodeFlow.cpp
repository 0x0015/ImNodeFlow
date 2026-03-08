#include "ImNodeFlow.h"
#include <algorithm>

namespace ImFlow {
    // -----------------------------------------------------------------------------------------------------------------
    // LINK

    // Helper function to check if a point is near a line segment
    static bool pointNearLineSegment(const ImVec2& p, const ImVec2& a, const ImVec2& b, float threshold) {
        ImVec2 ab = ImVec2(b.x - a.x, b.y - a.y);
        ImVec2 ap = ImVec2(p.x - a.x, p.y - a.y);
        float ab_len_sq = ab.x * ab.x + ab.y * ab.y;
        if (ab_len_sq < 0.001f) return false;
        float t = (ap.x * ab.x + ap.y * ab.y) / ab_len_sq;
        t = ImClamp(t, 0.0f, 1.0f);
        ImVec2 closest = ImVec2(a.x + t * ab.x, a.y + t * ab.y);
        float dist_sq = (p.x - closest.x) * (p.x - closest.x) + (p.y - closest.y) * (p.y - closest.y);
        return dist_sq < (threshold * threshold);
    }

    int Link::addWaypoint(const ImVec2& pos) {
        // pos is in grid coordinates
        // start/end from pinPoint() are in screen coordinates - convert to grid
        ImVec2 startScreen = m_left->pinPoint();
        ImVec2 endScreen = m_right->pinPoint();
        ImVec2 start = m_inf->screen2grid(startScreen);
        ImVec2 end = m_inf->screen2grid(endScreen);
        
        // Build list of all points in grid coordinates
        std::vector<ImVec2> points;
        points.push_back(start);
        for (const auto& wp : m_waypoints) {
            points.push_back(wp.pos);
        }
        points.push_back(end);

        // Find which segment the position is closest to
        int bestSegment = -1;
        float bestDist = FLT_MAX;
        for (size_t i = 0; i < points.size() - 1; i++) {
            ImVec2 a = points[i];
            ImVec2 b = points[i + 1];
            // Project pos onto segment [a, b]
            ImVec2 ab = ImVec2(b.x - a.x, b.y - a.y);
            ImVec2 ap = ImVec2(pos.x - a.x, pos.y - a.y);
            float ab_len_sq = ab.x * ab.x + ab.y * ab.y;
            if (ab_len_sq < 0.001f) continue;
            float t = (ap.x * ab.x + ap.y * ab.y) / ab_len_sq;
            t = ImClamp(t, 0.0f, 1.0f);
            ImVec2 closest = ImVec2(a.x + t * ab.x, a.y + t * ab.y);
            float dist = sqrtf((pos.x - closest.x) * (pos.x - closest.x) + (pos.y - closest.y) * (pos.y - closest.y));
            if (dist < bestDist) {
                bestDist = dist;
                bestSegment = i;
            }
        }

        Waypoint wp;
        wp.pos = pos;
        wp.posTarget = pos;  // Initialize target to same position
        if (bestSegment >= 0 && bestSegment < (int)m_waypoints.size()) {
            auto it = m_waypoints.insert(m_waypoints.begin() + bestSegment + 1, wp);
	    if((*m_inf).onLinkWaypointCreateHook)
		    (*m_inf).onLinkWaypointCreateHook(*it);
            return bestSegment + 1;
        } else {
            m_waypoints.push_back(wp);
	    if((*m_inf).onLinkWaypointCreateHook)
		    (*m_inf).onLinkWaypointCreateHook(m_waypoints.back());
            return (int)m_waypoints.size() - 1;
        }
    }

    void Link::removeWaypoint(int index) {
        if (index >= 0 && index < (int)m_waypoints.size()) {
	    if((*m_inf).onLinkWaypointDestroyHook)
		    (*m_inf).onLinkWaypointDestroyHook(m_waypoints[index]);
            m_waypoints.erase(m_waypoints.begin() + index);
        }
    }

    void Link::setWaypoints(const std::vector<ImVec2>& positions) {
        m_waypoints.clear();
        for (const auto& pos : positions) {
            Waypoint wp;
            wp.pos = pos;
            wp.posTarget = pos;  // Initialize target to same position
            m_waypoints.push_back(wp);
        }
    }

    int Link::getHoveredWaypoint() const {
        ImVec2 mousePos = m_inf->screen2grid(ImGui::GetMousePos());
        for (int i = 0; i < (int)m_waypoints.size(); i++) {
            float dist = sqrtf((mousePos.x - m_waypoints[i].pos.x) * (mousePos.x - m_waypoints[i].pos.x) +
                               (mousePos.y - m_waypoints[i].pos.y) * (mousePos.y - m_waypoints[i].pos.y));
            if (dist < Waypoint::HOVER_RADIUS) {
                return i;
            }
        }
        return -1;
    }

    void Link::update() {
        // start/end from pinPoint() are in screen coordinates - convert to grid
        ImVec2 startScreen = m_left->pinPoint();
        ImVec2 endScreen = m_right->pinPoint();
        ImVec2 start = m_inf->screen2grid(startScreen);
        ImVec2 end = m_inf->screen2grid(endScreen);
        
        float thickness = m_left->getStyle()->extra.link_thickness;
        bool mouseClickState = m_inf->getSingleUseClick();
        ImVec2 mousePos = ImGui::GetMousePos();
        ImVec2 mouseGridPos = m_inf->screen2grid(mousePos);

        // Build list of all points (start, waypoints, end) - all in GRID coordinates
        std::vector<ImVec2> points;
        points.push_back(start);
        for (auto& wp : m_waypoints) {
            points.push_back(wp.pos);
        }
        points.push_back(end);

        // Helper to normalize a vector
        auto normalize = [](const ImVec2& v) -> ImVec2 {
            float len = sqrt(v.x * v.x + v.y * v.y);
            if (len < 0.001f) return ImVec2(0, 0);
            return ImVec2(v.x / len, v.y / len);
        };
        
        // Calculate tangents for each segment based on neighboring points
        // tangent1[i] = outgoing tangent at points[i]
        // tangent2[i] = incoming tangent at points[i+1]
        std::vector<ImVec2> tangent1(points.size() - 1);
        std::vector<ImVec2> tangent2(points.size() - 1);
        
        for (size_t i = 0; i < points.size() - 1; i++) {
            ImVec2 curr = points[i];
            ImVec2 next = points[i + 1];
            
            // For the first point (output pin), use pin's tangent direction (accounts for flip)
            if (i == 0) {
                tangent1[i] = m_left->getTangentDirection();
            } else {
                // For waypoints, use direction from previous to next point for smooth curves
                ImVec2 prev = points[i - 1];
                tangent1[i] = normalize(ImVec2(next.x - prev.x, next.y - prev.y));
            }
            
            // For the last point (input pin), use pin's tangent direction (accounts for flip)
            if (i == points.size() - 2) {
                tangent2[i] = m_right->getTangentDirection();
            } else {
                // For waypoints, use direction from current to next-next point
                ImVec2 nextnext = points[i + 2];
                tangent2[i] = normalize(ImVec2(nextnext.x - curr.x, nextnext.y - curr.y));
            }
        }

        // Check if link is hovered (any segment) - use screen coordinates for collision
        m_hovered = false;
        for (size_t i = 0; i < points.size() - 1; i++) {
            bool hit = false;
            // Always use tangent-aware collision to respect pin directions (flip mode)
            hit = smart_bezier_collider_with_tangents(mousePos, 
                m_inf->grid2screen(points[i]), m_inf->grid2screen(points[i + 1]),
                tangent1[i], tangent2[i], 2.5);
            if (hit) {
                m_hovered = true;
                thickness = m_left->getStyle()->extra.link_hovered_thickness;
                if (mouseClickState && getHoveredWaypoint() < 0) {
                    m_inf->consumeSingleUseClick();
                    m_selected = true;
                }
                break;
            }
        }

        // Handle waypoint interactions
        int hoveredWp = getHoveredWaypoint();
        
        // Update hovered waypoint
        for (size_t i = 0; i < m_waypoints.size(); i++) {
            m_waypoints[i].hovered = (i == hoveredWp);
        }

        // Start dragging waypoint
        if (hoveredWp >= 0 && mouseClickState) {
            m_draggedWaypointIndex = hoveredWp;
            m_waypoints[hoveredWp].dragged = true;
            m_waypoints[hoveredWp].posTarget = m_waypoints[hoveredWp].pos;  // Initialize target
            m_inf->consumeSingleUseClick();
        }

        // Drag waypoint
        if (m_draggedWaypointIndex >= 0) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                float step = m_inf->getStyle().grid_size / m_inf->getStyle().grid_subdivisions;
                // Apply delta to target position (not snapped)
                m_waypoints[m_draggedWaypointIndex].posTarget += m_inf->getScreenSpaceDelta();
                // Snap to grid for display
                m_waypoints[m_draggedWaypointIndex].pos.x = round(m_waypoints[m_draggedWaypointIndex].posTarget.x / step) * step;
                m_waypoints[m_draggedWaypointIndex].pos.y = round(m_waypoints[m_draggedWaypointIndex].posTarget.y / step) * step;
            } else {
                m_waypoints[m_draggedWaypointIndex].dragged = false;
                m_waypoints[m_draggedWaypointIndex].posTarget = m_waypoints[m_draggedWaypointIndex].pos;  // Sync target
                m_draggedWaypointIndex = -1;
            }
        }

        // Delete waypoint on Delete key when hovered
        if (hoveredWp >= 0 && ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
            removeWaypoint(hoveredWp);
        }
        // Delete selected link on Delete key (only if no waypoint is hovered)
        else if (m_selected && hoveredWp < 0 && ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
            // Invalidate the link - this will cause it to be cleaned up
            m_right->deleteLink(this);
        }

        // Deselect on click elsewhere
        if (!ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_hovered && m_draggedWaypointIndex < 0) {
            m_selected = false;
        }

        // Draw the link segments
        ImU32 color = m_left->getStyle()->color;
        ImU32 outlineColor = m_left->getStyle()->extra.outline_color;

        // Rebuild points in case waypoints moved (still in GRID coords)
        points.clear();
        points.push_back(start);
        for (auto& wp : m_waypoints) {
            points.push_back(wp.pos);
        }
        points.push_back(end);
        
        // Recalculate tangents after points rebuild
        tangent1.resize(points.size() - 1);
        tangent2.resize(points.size() - 1);
        for (size_t i = 0; i < points.size() - 1; i++) {
            ImVec2 curr = points[i];
            ImVec2 next = points[i + 1];
            
            if (i == 0) {
                tangent1[i] = m_left->getTangentDirection();
            } else {
                ImVec2 prev = points[i - 1];
                tangent1[i] = normalize(ImVec2(next.x - prev.x, next.y - prev.y));
            }
            
            if (i == points.size() - 2) {
                tangent2[i] = m_right->getTangentDirection();
            } else {
                ImVec2 nextnext = points[i + 2];
                tangent2[i] = normalize(ImVec2(nextnext.x - curr.x, nextnext.y - curr.y));
            }
        }

        // Draw outline if selected - convert to screen for drawing
        if (m_selected) {
            for (size_t i = 0; i < points.size() - 1; i++) {
                smart_bezier_with_tangents(m_inf->grid2screen(points[i]), m_inf->grid2screen(points[i + 1]),
                             tangent1[i], tangent2[i], outlineColor,
                             thickness + m_left->getStyle()->extra.link_selected_outline_thickness);
            }
        }

        // Draw segments - convert to screen for drawing
        for (size_t i = 0; i < points.size() - 1; i++) {
            smart_bezier_with_tangents(m_inf->grid2screen(points[i]), m_inf->grid2screen(points[i + 1]),
                         tangent1[i], tangent2[i], color, thickness);
        }

        // Draw waypoints - convert to screen for drawing
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (auto& wp : m_waypoints) {
            ImVec2 screenPos = m_inf->grid2screen(wp.pos);
            float radius = Waypoint::RADIUS;
            if (wp.hovered || wp.dragged) {
                radius = Waypoint::HOVER_RADIUS;
            }
            draw_list->AddCircleFilled(screenPos, radius, color);
            if (wp.hovered || wp.dragged) {
                draw_list->AddCircle(screenPos, radius, outlineColor, 12, 2.0f);
            }
        }
    }

    Link::~Link(){
	if(!m_left) return;
        m_left->deleteLink(this);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // BASE NODE

    bool BaseNode::isHovered() {
        ImVec2 paddingTL = {m_style->padding.x, m_style->padding.y};
        ImVec2 paddingBR = {m_style->padding.z, m_style->padding.w};
        return ImGui::IsMouseHoveringRect(m_inf->grid2screen(m_pos - paddingTL),
                                          m_inf->grid2screen(m_pos + m_size + paddingBR)) && !m_inf->getDisabled();
    }

    void BaseNode::update() {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImGui::PushID(this);
        bool mouseClickState = m_inf->getSingleUseClick();
        ImVec2 offset = m_inf->grid2screen({0.f, 0.f});
        ImVec2 paddingTL = {m_style->padding.x, m_style->padding.y};
        ImVec2 paddingBR = {m_style->padding.z, m_style->padding.w};

        draw_list->ChannelsSetCurrent(1); // Foreground
        ImGui::SetCursorScreenPos(offset + m_pos);

        ImGui::BeginGroup();

        // Header
        ImGui::BeginGroup();
        ImGui::TextColored(m_style->header_title_color, "%s", m_title.c_str());
        ImGui::Spacing();
        ImGui::EndGroup();
        float headerH = ImGui::GetItemRectSize().y;
        float titleW = ImGui::GetItemRectSize().x;

        // Inputs
        if (!m_ins.empty()) {
            ImGui::BeginGroup();
            for (auto &p: m_ins) {
                p->setPos(ImGui::GetCursorPos());
                p->update();
            }
            for (auto &p: m_dynamicIns) {
                if (p.first == 1) {
                    p.second->setPos(ImGui::GetCursorPos());
                    p.second->update();
                    p.first = 0;
                }
            }
            ImGui::EndGroup();
            ImGui::SameLine();
        }

        // Content
        ImGui::BeginGroup();
        draw();
        ImGui::Dummy(ImVec2(0.f, 0.f));
        ImGui::EndGroup();
        ImGui::SameLine();

        // Outputs
        float maxW = 0.0f;
        for (auto &p: m_outs) {
            float w = p->calcWidth();
            if (w > maxW)
                maxW = w;
        }
        for (auto &p: m_dynamicOuts) {
            float w = p.second->calcWidth();
            if (w > maxW)
                maxW = w;
        }
        ImGui::BeginGroup();
        for (auto &p: m_outs) {
            // FIXME: This looks horrible
            if ((m_pos + ImVec2(titleW, 0) + m_inf->getGrid().scroll()).x <
                ImGui::GetCursorPos().x + ImGui::GetWindowPos().x + maxW)
                p->setPos(ImGui::GetCursorPos() + ImGui::GetWindowPos() + ImVec2(maxW - p->calcWidth(), 0.f));
            else
                p->setPos(ImVec2((m_pos + ImVec2(titleW - p->calcWidth(), 0) + m_inf->getGrid().scroll()).x,
                                 ImGui::GetCursorPos().y + ImGui::GetWindowPos().y));
            p->update();
        }
        for (auto &p: m_dynamicOuts) {
            // FIXME: This looks horrible
            if ((m_pos + ImVec2(titleW, 0) + m_inf->getGrid().scroll()).x <
                ImGui::GetCursorPos().x + ImGui::GetWindowPos().x + maxW)
                p.second->setPos(
                        ImGui::GetCursorPos() + ImGui::GetWindowPos() + ImVec2(maxW - p.second->calcWidth(), 0.f));
            else
                p.second->setPos(
                        ImVec2((m_pos + ImVec2(titleW - p.second->calcWidth(), 0) + m_inf->getGrid().scroll()).x,
                               ImGui::GetCursorPos().y + ImGui::GetWindowPos().y));
            p.second->update();
            p.first -= 1;
        }

        ImGui::EndGroup();

        ImGui::EndGroup();
        m_size = ImGui::GetItemRectSize();
        ImVec2 headerSize = ImVec2(m_size.x + paddingBR.x, headerH);

        // Background
        draw_list->ChannelsSetCurrent(0);
        draw_list->AddRectFilled(offset + m_pos - paddingTL, offset + m_pos + m_size + paddingBR, m_style->bg,
                                 m_style->radius);
        draw_list->AddRectFilled(offset + m_pos - paddingTL, offset + m_pos + headerSize, m_style->header_bg,
                                 m_style->radius, ImDrawFlags_RoundCornersTop);

        ImU32 col = m_style->border_color;
        float thickness = m_style->border_thickness;
        ImVec2 ptl = paddingTL;
        ImVec2 pbr = paddingBR;
        if (m_selected) {
            col = m_style->border_selected_color;
            thickness = m_style->border_selected_thickness;
        }
        if (thickness < 0.f) {
            ptl.x -= thickness / 2;
            ptl.y -= thickness / 2;
            pbr.x -= thickness / 2;
            pbr.y -= thickness / 2;
            thickness *= -1.f;
        }
        draw_list->AddRect(offset + m_pos - ptl, offset + m_pos + m_size + pbr, col, m_style->radius, 0, thickness);


        if (ImGui::IsWindowHovered() && !ImGui::IsKeyDown(ImGuiKey_LeftCtrl) &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_inf->on_selected_node())
            selected(false);

        if (isHovered()) {
            m_inf->hoveredNode(this);
            if (mouseClickState) {
                selected(true);
                m_inf->consumeSingleUseClick();
            }
        }

        if (ImGui::IsWindowFocused() && ImGui::IsKeyPressed(ImGuiKey_Delete) && !ImGui::IsAnyItemActive() && isSelected() && !m_inf->getDisabled())
            destroy();

        bool onHeader = ImGui::IsMouseHoveringRect(offset + m_pos - paddingTL, offset + m_pos + headerSize);
        if (onHeader && mouseClickState && !m_inf->getDisabled()) {
            m_inf->consumeSingleUseClick();
            m_dragged = true;
            m_inf->draggingNode(true);
        }
        if (m_dragged || (m_selected && m_inf->isNodeDragged())) {
            float step = m_inf->getStyle().grid_size / m_inf->getStyle().grid_subdivisions;
            m_posTarget += m_inf->getScreenSpaceDelta();
            // "Slam" The position
            m_pos.x = round(m_posTarget.x / step) * step;
            m_pos.y = round(m_posTarget.y / step) * step;

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                m_dragged = false;
                m_inf->draggingNode(false);
                m_posTarget = m_pos;
            }
        }
        ImGui::PopID();

        // Deleting dead pins
        m_dynamicIns.erase(std::remove_if(m_dynamicIns.begin(), m_dynamicIns.end(),
                                          [](const std::pair<int, std::shared_ptr<Pin>> &p) { return p.first == 0; }),
                           m_dynamicIns.end());
        m_dynamicOuts.erase(std::remove_if(m_dynamicOuts.begin(), m_dynamicOuts.end(),
                                           [](const std::pair<int, std::shared_ptr<Pin>> &p) { return p.first == 0; }),
                            m_dynamicOuts.end());
    }

    // -----------------------------------------------------------------------------------------------------------------
    // HANDLER

    int ImNodeFlow::m_instances = 0;

    bool ImNodeFlow::on_selected_node() {
        return std::any_of(m_nodes.begin(), m_nodes.end(),
                           [](const auto &n) { return n.second->isSelected() && n.second->isHovered(); });
    }

    bool ImNodeFlow::on_free_space() {
        return std::all_of(m_nodes.begin(), m_nodes.end(),
                           [](const auto &n) { return !n.second->isHovered(); })
               && std::all_of(m_links.begin(), m_links.end(),
                              [](const auto &l) {
			          auto link = l.lock();
				  return !link || (!link->isHovered() && link->getHoveredWaypoint() < 0);
			      });
    }

    std::weak_ptr<Link> ImNodeFlow::getHoveredLink() {
        for (auto& l : m_links) {
            auto link = l.lock();
            if (link && link->isHovered()) {
                return l;
            }
        }
        return std::weak_ptr<Link>();
    }

    int ImNodeFlow::addWaypointToLink(std::shared_ptr<Link> link, const ImVec2& pos) {
        if (link) {
            return link->addWaypoint(pos);
        }
        return -1;
    }

    ImVec2 ImNodeFlow::screen2grid( const ImVec2 & p )
    {
        if ( ImGui::GetCurrentContext() == m_context.getRawContext() )
            return p - m_context.scroll();
        return ( p - m_context.origin() ) / m_context.scale() - m_context.scroll();
    }

    ImVec2 ImNodeFlow::grid2screen( const ImVec2 & p )
    {
        if ( ImGui::GetCurrentContext() == m_context.getRawContext() )
            return p + m_context.scroll();
        return ( p + m_context.scroll() ) * m_context.scale() + m_context.origin();
    }

    void ImNodeFlow::addLink(std::shared_ptr<Link> &link) {
        m_links.push_back(link);
    }

    void ImNodeFlow::update(bool disabled) {
        // Updating looping stuff
        m_hovering = nullptr;
        m_hoveredNode = nullptr;
	m_hoveredLink.reset(); //reset hovered link every frame
        m_draggingNode = m_draggingNodeNext;
        m_singleUseClick = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
	m_doubleUseClick = ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
	m_disabled = disabled;

        // Create child canvas
        m_context.begin();
        ImGui::GetIO().IniFilename = nullptr;
	
	//if disabled draw a transparent input-blocking window over the entire thing.  This blocks most input, though doesn't seem to block node dragging/selection
	if(m_disabled){
		ImGui::SetNextWindowPos({0, 0});
		ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
		ImGui::Begin("##ImNodeFlowInputBlocker", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
		ImGui::End();
	}

        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        // Display grid
        ImVec2 gridSize = ImGui::GetWindowSize();
        float subGridStep = m_style.grid_size / m_style.grid_subdivisions;
        for (float x = fmodf(m_context.scroll().x, m_style.grid_size); x < gridSize.x; x += m_style.grid_size)
            draw_list->AddLine(ImVec2(x, 0.0f), ImVec2(x, gridSize.y), m_style.colors.grid);
        for (float y = fmodf(m_context.scroll().y, m_style.grid_size); y < gridSize.y; y += m_style.grid_size)
            draw_list->AddLine(ImVec2(0.0f, y), ImVec2(gridSize.x, y), m_style.colors.grid);
        if (m_context.scale() > 0.7f) {
            for (float x = fmodf(m_context.scroll().x, subGridStep); x < gridSize.x; x += subGridStep)
                draw_list->AddLine(ImVec2(x, 0.0f), ImVec2(x, gridSize.y), m_style.colors.subGrid);
            for (float y = fmodf(m_context.scroll().y, subGridStep); y < gridSize.y; y += subGridStep)
                draw_list->AddLine(ImVec2(0.0f, y), ImVec2(gridSize.x, y), m_style.colors.subGrid);
        }

        // Update and draw nodes
        // TODO: I don't like this
        draw_list->ChannelsSplit(2);
        for (auto &node: m_nodes) {
		node.second->m_inf = this;
		node.second->update();
	}
        // Remove "toDelete" nodes
	destroyDestroyedNodes();
        draw_list->ChannelsMerge();
        for (auto &node: m_nodes) { node.second->updatePublicStatus(); }

        // Update and draw links
        for (auto &l: m_links) {
		if(!l.expired()){
			l.lock()->m_inf = this;
			l.lock()->update();
		}
	}

        // Links drop-off
        if (m_dragOut && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            if (!m_hovering) {
                if (on_free_space() && m_droppedLinkPopUp) {
                    if (m_droppedLinkPupUpComboKey == ImGuiKey_None || ImGui::IsKeyDown(m_droppedLinkPupUpComboKey)) {
                        m_droppedLinkLeft = m_dragOut;
                        ImGui::OpenPopup("DroppedLinkPopUp");
                    }
                }
            } else
                m_dragOut->createLink(m_hovering);
        }

        // Links drag-out
        if (!m_draggingNode && m_hovering && !m_dragOut && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_disabled)
            m_dragOut = m_hovering;
        if (m_dragOut) {
            if (m_dragOut->getType() == PinType_Output)
                smart_bezier(m_dragOut->pinPoint(), ImGui::GetMousePos(), m_dragOut->getStyle()->color,
                             m_dragOut->getStyle()->extra.link_dragged_thickness);
            else
                smart_bezier(ImGui::GetMousePos(), m_dragOut->pinPoint(), m_dragOut->getStyle()->color,
                             m_dragOut->getStyle()->extra.link_dragged_thickness);

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                m_dragOut = nullptr;
        }

	// Double clicking on link adds waypoint
	if(m_doubleUseClick && ImGui::IsWindowHovered() && getHoveredLink().lock()){
		getHoveredLink().lock()->addWaypoint(screen2grid(ImGui::GetMousePos()));
	}

        // Right-click PopUp
        if (m_rightClickPopUp && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && ImGui::IsWindowHovered()) {
            m_hoveredNodeAux = m_hoveredNode;
	    m_hoveredLinkAux = getHoveredLink().lock();
            ImGui::OpenPopup("RightClickPopUp");
        }
        if (ImGui::BeginPopup("RightClickPopUp")) {
            m_rightClickPopUp(m_hoveredNodeAux);
            ImGui::EndPopup();
        }

        // Dropped Link PopUp
        if (ImGui::BeginPopup("DroppedLinkPopUp")) {
            m_droppedLinkPopUp(m_droppedLinkLeft);
            ImGui::EndPopup();
        }

        // Removing dead Links
        m_links.erase(std::remove_if(m_links.begin(), m_links.end(),
                                     [](const std::weak_ptr<Link> &l) { return l.expired(); }), m_links.end());

        // Clearing recursion blacklist
        m_pinRecursionBlacklist.clear();

        m_context.end();
    }

    void ImNodeFlow::destroyDestroyedNodes(){
	std::erase_if(m_nodes, [](const auto& m_node){
		if(!m_node.second || m_node.second->toDestroy())
			return true;
		return false;
	});
    }
}
