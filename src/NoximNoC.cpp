/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the Network-on-Chip
 */

#include "NoximNoC.h"

void NoximNoC::buildMesh()
{
    // Check for routing table availability
    if (NoximGlobalParams::routing_algorithm == ROUTING_TABLE_BASED)
	assert(grtable.load(NoximGlobalParams::routing_table_filename));
    
    // Check for traffic table availability
    if (NoximGlobalParams::traffic_distribution == TRAFFIC_TABLE_BASED)
	assert(gttable.load(NoximGlobalParams::traffic_table_filename));

    // dummy NoximNoP_data structure
    NoximNoP_data tmp_NoP;
    tmp_NoP.sender_id = NOT_VALID;
    for (int i = 0; i < DIRECTIONS; i++) {
	tmp_NoP.channel_status_neighbor[i].free_slots = NOT_VALID;
	tmp_NoP.channel_status_neighbor[i].available = false;
    }

    // Create the mesh as a matrix of tiles
    for (int i = 0; i < NoximGlobalParams::mesh_dim_x; i++) {
	for (int j = 0; j < NoximGlobalParams::mesh_dim_y; j++) {
	    for (int k = 0; k < NoximGlobalParams::mesh_dim_z; ++k) {
		// Create the single Tile with a proper name
		int tile_id = coord2Id (i, j, k);
		char tile_name [32];
		sprintf(tile_name, "Tile[%02d][%02d][%02d]", i, j, k);
		NoximTile* tile = t[i][j][k] = new NoximTile(tile_name);

		
		// Tell to the router its coordinates
		tile->r->configure(tile_id,
				   NoximGlobalParams::stats_warm_up_time,
				   NoximGlobalParams::buffer_depth,
				   grtable);

		// Tell to the PE its coordinates
		tile->pe->local_id = tile_id;
		tile->pe->traffic_table = &gttable;	// Needed to choose destination
		tile->pe->never_transmit = (gttable.occurrencesAsSource(tile->pe->local_id) == 0);

		// Map clock and reset
		tile->clock(clock);
		tile->reset(reset);

		// Map Rx signals
		for (int dir = 0; dir < DIRECTIONS; ++ dir)
		{
		    if (dir == DIRECTION_DOWN && !NoximGlobalParams::has_tsv [tile_id])
		    {
			/* Cut connection cuz no TSV exist. */
			tile->req_rx [dir] (*new sc_signal <bool>);
			tile->ack_rx [dir] (*new sc_signal <bool>);
			tile->flit_rx [dir] (*new sc_signal <NoximFlit>);
			tile->req_tx [dir] (*new sc_signal <bool>);
			tile->ack_tx [dir] (*new sc_signal <bool>);
			tile->flit_tx [dir] (*new sc_signal <NoximFlit>);
			tile->free_slots [dir] (*new sc_signal <int>);
			tile->free_slots_neighbor [dir] (*new sc_signal <int>);
			tile->NoP_data_in [dir] (*new sc_signal <NoximNoP_data>);
			tile->NoP_data_out [dir] (*new sc_signal <NoximNoP_data>);
		    }
		    else
		    {
			int opd = opposite (dir);
			int si = dir == DIRECTION_EAST  ? i + 1 : i,
			    sj = dir == DIRECTION_SOUTH ? j + 1 : j,
			    sk = dir == DIRECTION_UP    ? k + 1 : k;
		    
			tile->req_rx  [dir] (req_to_dir  [opd][si][sj][sk]);
			tile->flit_rx [dir] (flit_to_dir [opd][si][sj][sk]);
			tile->ack_rx  [dir] (ack_to_dir  [dir][si][sj][sk]);

			tile->req_tx  [dir] (req_to_dir  [dir][si][sj][sk]);
			tile->flit_tx [dir] (flit_to_dir [dir][si][sj][sk]);
			tile->ack_tx  [dir] (ack_to_dir  [opd][si][sj][sk]);

			// Map buffer level signals (analogy with req_tx/rx port mapping)
			tile->free_slots [dir] (free_slots_to_dir [dir][si][sj][sk]);
			tile->free_slots_neighbor [dir] (free_slots_to_dir [opd][si][sj][sk]);

			// NoP 
			tile->NoP_data_out[dir] (NoP_data_to_dir [dir][si][sj][sk]);
			tile->NoP_data_in[dir] (NoP_data_to_dir [opd][si][sj][sk]);
		    }
		}
	    }
	}
    }

#if 1
    // Clear signals for borderline nodes
    for (int i = 0; i <= NoximGlobalParams::mesh_dim_x; i++) {
	for (int k = 0; k <= NoximGlobalParams::mesh_dim_z; k++) {
	    req_to_dir [DIRECTION_SOUTH][i][0][k] = 0;
	    ack_to_dir [DIRECTION_NORTH][i][0][k] = 0;
	    req_to_dir [DIRECTION_NORTH][i][NoximGlobalParams::mesh_dim_y][k] = 0;
	    ack_to_dir [DIRECTION_SOUTH][i][NoximGlobalParams::mesh_dim_y][k] = 0;
	
	    free_slots_to_dir [DIRECTION_SOUTH][i][0][k].write(NOT_VALID);
	    free_slots_to_dir [DIRECTION_NORTH][i][NoximGlobalParams::mesh_dim_y][k].write(NOT_VALID);
	    
	    NoP_data_to_dir [DIRECTION_SOUTH][i][0][k].write(tmp_NoP);
	    NoP_data_to_dir [DIRECTION_NORTH][i][NoximGlobalParams::mesh_dim_y][k].write(tmp_NoP);
	}
    }

    for (int j = 0; j <= NoximGlobalParams::mesh_dim_y; j++) {
	for (int k = 0; k <= NoximGlobalParams::mesh_dim_z; k++) {
	    req_to_dir [DIRECTION_EAST][0][j][k] = 0;
	    ack_to_dir [DIRECTION_WEST][0][j][k] = 0;
	    req_to_dir [DIRECTION_WEST][NoximGlobalParams::mesh_dim_x][j][k] = 0;
	    ack_to_dir [DIRECTION_EAST][NoximGlobalParams::mesh_dim_x][j][k] = 0;

	    free_slots_to_dir [DIRECTION_EAST][0][j][k].write(NOT_VALID);
	    free_slots_to_dir [DIRECTION_WEST][NoximGlobalParams::mesh_dim_x][j][k].write(NOT_VALID);
	    
	    NoP_data_to_dir [DIRECTION_EAST][0][j][k].write(tmp_NoP);
	    NoP_data_to_dir [DIRECTION_WEST][NoximGlobalParams::mesh_dim_x][j][k].write(tmp_NoP);
	}
    }

    for (int i = 0; i <= NoximGlobalParams::mesh_dim_x; i++) {
	for (int j = 0; j <= NoximGlobalParams::mesh_dim_y; j++) {
	    req_to_dir [DIRECTION_UP][i][j][0] = 0;
	    ack_to_dir [DIRECTION_DOWN][i][j][0] = 0;
	    req_to_dir [DIRECTION_DOWN][i][j][NoximGlobalParams::mesh_dim_z] = 0;
	    ack_to_dir [DIRECTION_UP][i][j][NoximGlobalParams::mesh_dim_z] = 0;
	
	    free_slots_to_dir [DIRECTION_UP][i][j][0].write(NOT_VALID);
	    free_slots_to_dir [DIRECTION_DOWN][i][j][NoximGlobalParams::mesh_dim_z].write(NOT_VALID);
	    
	    NoP_data_to_dir [DIRECTION_UP][i][j][0].write(tmp_NoP);
	    NoP_data_to_dir [DIRECTION_DOWN][i][j][NoximGlobalParams::mesh_dim_z].write(tmp_NoP);
	}
    }

    // invalidate reservation table entries for non-exhistent channels
    for (int i = 0; i < NoximGlobalParams::mesh_dim_x; i++) {
	for (int k = 0; k < NoximGlobalParams::mesh_dim_z; ++k) {
	    t[i][0][k]->r->reservation_table.invalidate(DIRECTION_NORTH);
	    t[i][NoximGlobalParams::mesh_dim_y - 1][k]->r->reservation_table.invalidate(DIRECTION_SOUTH);
	}
    }
    for (int j = 0; j < NoximGlobalParams::mesh_dim_y; j++) {
	for (int k = 0; k < NoximGlobalParams::mesh_dim_z; ++k) {
	    t[0][j][k]->r->reservation_table.invalidate(DIRECTION_WEST);
	    t[NoximGlobalParams::mesh_dim_x - 1][j][k]->r->reservation_table.invalidate(DIRECTION_EAST);
	}
    }
    for (int i = 0; i < NoximGlobalParams::mesh_dim_x; i++) {
	for (int j = 0; j < NoximGlobalParams::mesh_dim_y; ++j) {
	    t[i][j][0]->r->reservation_table.invalidate(DIRECTION_DOWN);
	    t[i][j][NoximGlobalParams::mesh_dim_z - 1]->r->reservation_table.invalidate(DIRECTION_UP);
	}
    }
#endif
}
    
NoximTile *NoximNoC::searchNode(const int id) const
{
    // TODO: id has a coordinates inverse, motherfucker.
    for (int i = 0; i < NoximGlobalParams::mesh_dim_x; i++)
	for (int j = 0; j < NoximGlobalParams::mesh_dim_y; j++)
	    for (int k = 0; k < NoximGlobalParams::mesh_dim_z; ++k)
		if (t[i][j][k]->r->local_id == id)
		    return t[i][j][k];

    return false;
}
