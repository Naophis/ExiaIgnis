// Pure types/constants shared between the server-only lib/test-templates.ts
// (which touches node:fs) and client components. Keep this file free of any
// Node built-ins so client bundles importing TEST_TEMPLATE_KEYS don't pull
// fs/path/crypto into the browser bundle.

export const TEST_TEMPLATE_KEYS = [
  "v_max",
  "accl",
  "decel",
  "dia_accl",
  "dia_decel",
  "dist",
  "suction_active",
  // "mode" lives at the top level of system.yaml (outside test:); the rest
  // are inside test: alongside the keys above. applyTestTemplateToSystemYaml
  // searches the whole file per key, so mixing scopes here is fine.
  "mode",
  "file_idx",
  "sla_type",
  "sla_type2",
  "sla_return",
  "ignore_opp_sen",
  "search_mode",
] as const;

export type TestTemplateKey = (typeof TEST_TEMPLATE_KEYS)[number];
export type TestTemplateValues = Partial<Record<TestTemplateKey, number>>;

export interface TestTemplate {
  id: string;
  name: string;
  values: TestTemplateValues;
}

export interface NamedOption {
  label: string;
  value: number;
}

// Shared turn-type numbering, per system.yaml's own sla_type comment
// cheat-sheet (lines ~179-187) - Kojima/7 is a dead, non-functional turn
// type and deliberately excluded.
export const SLA_TYPE_OPTIONS: NamedOption[] = [
  { label: "Normal", value: 1 },
  { label: "Oval", value: 2 },
  { label: "Large", value: 3 },
  { label: "Dia45", value: 4 },
  { label: "Dia135", value: 5 },
  { label: "Dia90", value: 6 },
  { label: "Dia45_2", value: 8 },
  { label: "Dia135_2", value: 9 },
];

// sla_type2 (復路ターン種類) only ever uses the diagonal-return subset.
export const SLA_TYPE2_OPTIONS: NamedOption[] = [
  { label: "Dia90", value: 6 },
  { label: "Dia45_2", value: 8 },
  { label: "Dia135_2", value: 9 },
];

export const SLA_RETURN_OPTIONS: NamedOption[] = [
  { label: "しない", value: 0 },
  { label: "する", value: 1 },
];

export const TEST_TEMPLATE_KEY_OPTIONS: Partial<Record<TestTemplateKey, NamedOption[]>> = {
  sla_type: SLA_TYPE_OPTIONS,
  sla_type2: SLA_TYPE2_OPTIONS,
  sla_return: SLA_RETURN_OPTIONS,
};
