#!/usr/bin/env bash

# we need bash 4 for associative arrays
if [ "${BASH_VERSION%%[^0-9]*}" -lt "4" ]; then
  echo "BASH VERSION < 4: ${BASH_VERSION}" >&2
  exit 1
fi

echo github.action: ${{github.action}};
echo github.action_path: ${{github.action_path}};
echo github.action_ref: ${{github.action_ref}};
echo github.action_repository: ${{github.action_repository}};
echo github.actor: ${{github.actor}};
echo github.api_url: ${{github.api_url}};
echo github.base_ref: ${{github.base_ref}};
echo github.env: ${{github.env}};
echo github.event: ${{github.event}};
echo github.event_name: ${{github.event_name}};
echo github.event_path: ${{github.event_path}};
echo github.graphql_url: ${{github.graphql_url}};
echo github.head_ref: ${{github.head_ref}};
echo github.job: ${{github.job}};
echo github.ref: ${{github.ref}};
echo github.ref_name: ${{github.ref_name}};
echo github.ref_protected: ${{github.ref_protected}};
echo github.ref_type: ${{github.ref_type}};
echo github.path: ${{github.path}};
echo github.repository: ${{github.repository}};
echo github.repository_owner: ${{github.repository_owner}};
echo github.repositoryUrl: ${{github.repositoryUrl}};
echo github.retention_days: ${{github.retention_days}};
echo github.run_id: ${{github.run_id}};
echo github.run_number: ${{github.run_number}};
echo github.run_attempt: ${{github.run_attempt}};
echo github.server_url: ${{github.server_url}};
echo github.sha: ${{github.sha}};
echo github.token: ${{github.token}};
echo github.workflow: ${{github.workflow}};
echo github.workspace: ${{github.workspace}};
