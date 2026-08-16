import { NextRequest, NextResponse } from "next/server";
import {
  applyTestTemplateToSystemYaml,
  readActiveValues,
  TEST_TEMPLATE_KEYS,
  type TestTemplateValues,
} from "@/lib/test-templates";

export const runtime = "nodejs";

// Ad-hoc apply, bypassing the named-template list entirely: for workflows
// like sweeping sla_type 3..10 at a fixed file_idx, saving/deleting a named
// template per value tried would just clutter the template list.
export async function GET() {
  return NextResponse.json({ values: readActiveValues(TEST_TEMPLATE_KEYS) });
}

export async function POST(request: NextRequest) {
  const body = await request.json();
  const values = body?.values as TestTemplateValues | undefined;
  if (!values || Object.keys(values).length === 0) {
    return NextResponse.json({ error: "values is required" }, { status: 400 });
  }
  try {
    applyTestTemplateToSystemYaml(values);
    return NextResponse.json({ ok: true, values: readActiveValues(TEST_TEMPLATE_KEYS) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
