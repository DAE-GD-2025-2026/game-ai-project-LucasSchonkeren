#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
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
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	// TODO create the cells
	
	Cells.reserve(Rows * Cols);

	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float left = col * CellWidth - Width/2;
			float bottom = row * CellHeight - Height/2;

			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	// TODO Add the agent to the correct cell
	const FVector2D pos = Agent.GetPosition();
	int index = PositionToIndex(pos);

	if (index >= 0 && index < Cells.size())
	{
		Cells[index].Agents.push_back(&Agent);
	}
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this
	
	int oldIdx = PositionToIndex(OldPos);
	int newIdx = PositionToIndex(Agent.GetPosition());

	if (oldIdx == newIdx) return;

	if (oldIdx >= 0 && oldIdx < Cells.size())
	{
		Cells[oldIdx].Agents.remove(&Agent);
	}

	if (newIdx >= 0 && newIdx < Cells.size())
	{
		Cells[newIdx].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood
	
	NrOfNeighbors = 0;

	FVector2D pos = Agent.GetPosition();

	FRect queryRect;
	queryRect.Min = { pos.X - QueryRadius, pos.Y - QueryRadius };
	queryRect.Max = { pos.X + QueryRadius, pos.Y + QueryRadius };

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(cell.BoundingBox, queryRect))
			continue;

		for (ASteeringAgent* other : cell.Agents)
		{
			if (&Agent == other) continue;

			FVector2D to = other->GetPosition() - pos;

			if (to.SizeSquared() < QueryRadius * QueryRadius)
			{
				if (NrOfNeighbors < Neighbors.size())
				{
					Neighbors[NrOfNeighbors++] = other;
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	// TODO Render the cells with the number of agents inside of it
	
	for (const Cell& cell : Cells)
	{
		auto points = cell.GetRectPoints();

		for (int i = 0; i < 4; ++i)
		{
			FVector2D a = points[i];
			FVector2D b = points[(i + 1) % 4];

			DrawDebugLine(
				pWorld,
				FVector(a.X, a.Y, 0),
				FVector(b.X, b.Y, 0),
				FColor::White,
				false,
				-1.f,
				0,
				1.f
			);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	// TODO Calculate the index of the cell based on the position
	float shiftedX = Pos.X + SpaceWidth * 0.5f;
	float shiftedY = Pos.Y + SpaceHeight * 0.5f;

	int col = static_cast<int>(shiftedX / CellWidth);
	int row = static_cast<int>(shiftedY / CellHeight);

	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}