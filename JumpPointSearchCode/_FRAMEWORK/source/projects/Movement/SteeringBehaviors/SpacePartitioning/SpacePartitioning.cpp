#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
{
	m_CellWidth = m_SpaceWidth / m_NrOfCols;
	m_CellHeight = m_SpaceHeight / m_NrOfRows;

	for (int i{ 0 }; i < m_NrOfRows * m_NrOfCols; ++i)
	{
		m_Cells.push_back(Cell{ float(m_CellWidth * (i % cols)) - m_SpaceWidth * 0.5f, -float(m_CellHeight * floor((i) / cols)) + m_SpaceHeight * 0.5f - m_CellHeight, m_CellWidth, m_CellHeight });
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	auto oldPosIdx = PositionToIndex(oldPos);
	if (oldPosIdx != PositionToIndex(agent->GetPosition()))
	{
		m_Cells[oldPosIdx].agents.remove(agent);
		AddAgent(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius)
{
	m_NrOfNeighbors = 0;

	int topLeftIndex, topRightIndex, botLeftIndex;

	GetCellsToCheck(topLeftIndex, topRightIndex, botLeftIndex, agent, queryRadius);

	int colAmount{ topRightIndex - topLeftIndex + 1 };

	int rowAmount{ botLeftIndex - topLeftIndex };
	rowAmount /= m_NrOfCols;
	rowAmount += 1;

	for (int i{ topLeftIndex }; i <= topRightIndex; ++i)
	{
		for (int j{ 0 }; j < rowAmount; ++j)
		{
			for (auto neighbor : m_Cells[i+j*m_NrOfCols].agents)
			{
				if (neighbor != agent && (agent->GetPosition() - neighbor->GetPosition()).Magnitude() <= queryRadius)
				{
					m_Neighbors[m_NrOfNeighbors] = neighbor;
					++m_NrOfNeighbors;
				}
			}
		}
	}
}

void CellSpace::RenderCells() const
{
	for (auto cell : m_Cells)
	{
		std::string amount{ std::to_string(cell.agents.size()) };
		const char * pAmount{ amount.c_str() };

		Elite::Vector2 bottomLeft{ cell.boundingBox.bottomLeft };
		DEBUGRENDERER2D->DrawPolygon(&cell.GetRectPoints()[0], cell.GetRectPoints().size(), Elite::Color(1, 1, 1), 0.4f);

		bottomLeft.y += m_CellHeight;
		DEBUGRENDERER2D->DrawString(bottomLeft, pAmount);
	}

	auto left = m_OverlapRect.bottomLeft.x;
	auto bottom = m_OverlapRect.bottomLeft.y;
	auto width = m_OverlapRect.width;
	auto height = m_OverlapRect.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};
	DEBUGRENDERER2D->DrawPolygon(&rectPoints[0], rectPoints.size(), Elite::Color(1, 0, 0), 0.4f);

	int colAmount{ m_TopRightIdx - m_TopLeftIdx + 1 };

	int rowAmount{ m_BotLeftIdx - m_TopLeftIdx };
	rowAmount /= m_NrOfCols;
	rowAmount += 1;
	for (int i{ m_TopLeftIdx }; i <= m_TopRightIdx; ++i)
	{
		for (int j{ 0 }; j < rowAmount; ++j)
		{
			DEBUGRENDERER2D->DrawSolidPolygon(&m_Cells[i + j * m_NrOfCols].GetRectPoints()[0], m_Cells[i + j * m_NrOfCols].GetRectPoints().size(), Elite::Color(1, 0, 0), 0.4f);
		}
	}
}

void CellSpace::SetDebugValues(SteeringAgent* agent, float queryRadius)
{
	m_OverlapRect = Elite::Rect{ Elite::Vector2(agent->GetPosition().x - queryRadius, agent->GetPosition().y - queryRadius), queryRadius * 2.f, queryRadius * 2.f };
	GetCellsToCheck(m_TopLeftIdx, m_TopRightIdx, m_BotLeftIdx, agent, queryRadius);
}

void CellSpace::GetCellsToCheck(int& topLeftIdx, int& topRightIdx, int& botLeftIdx, SteeringAgent* agent, float queryRadius)
{
	Elite::Vector2 topLeft{ agent->GetPosition() };
	topLeft.x -= queryRadius;
	topLeft.y += queryRadius;
	if (topLeft.x < -m_SpaceWidth * 0.5f)
		topLeft.x = -m_SpaceWidth * 0.5f;
	topLeftIdx = PositionToIndex(topLeft);

	Elite::Vector2 topRight{ agent->GetPosition() };
	topRight.x += queryRadius;
	topRight.y += queryRadius;
	if (topRight.x > m_SpaceWidth * 0.5f)
		topRight.x = m_SpaceWidth * 0.5f;
	topRightIdx = PositionToIndex(topRight);

	Elite::Vector2 botLeft{ agent->GetPosition() };
	botLeft.x -= queryRadius;
	botLeft.y -= queryRadius;
	if (botLeft.y < -m_SpaceWidth * 0.5f)
		botLeft.y = -m_SpaceWidth * 0.5f;
	botLeftIdx = PositionToIndex(botLeft);
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	int index{};
	int col = int((pos.x + m_SpaceWidth * 0.5f) / m_CellWidth);

	if (col >= m_NrOfCols)
		col = m_NrOfCols - 1;
	if (col < 0)
		col = 0;

	int row = int((m_SpaceWidth * 0.5f - pos.y) / (m_CellHeight));
	if (row >= m_NrOfRows)
		row = m_NrOfRows - 1;
	if (row < 0)
		row = 0;

	index = int(col + (row * m_NrOfCols));
	return index;
}